"""
Meshtastic Driver for XPI

Handles real-time telemetry, commands, and neighbor tracking.
"""
import logging
import json
import time
from typing import Dict, Any, Callable, List
import meshtastic.serial_interface
import meshtastic.tcp_interface
from pubsub import pub # Meshtastic uses pyubsub for events

logger = logging.getLogger(__name__)

class MeshtasticDriver:
    def __init__(self, interface_type="serial", address="/dev/ttyUSB0"):
        self.interface_type = interface_type
        self.address = address
        self.interface = None
        
        # Состояние соседей (Neighbor DB)
        # ID -> { "last_seen": timestamp, "telemetry": {}, "snr": float }
        self.neighbors: Dict[str, Any] = {}
        
        # Коллбэки для внешних систем (например, ROS2)
        self.on_telemetry_received: List[Callable] = []
        self.on_command_received: List[Callable] = []

    def connect(self):
        try:
            if self.interface_type == "serial":
                self.interface = meshtastic.serial_interface.SerialInterface(self.address)
            else:
                self.interface = meshtastic.tcp_interface.TCPInterface(self.address)
            
            # Подписываемся на входящие пакеты от Meshtastic
            pub.subscribe(self._on_packet_received, "meshtastic.receive")
            logger.info(f"Successfully connected to Meshtastic @ {self.address}")
        except Exception as e:
            logger.error(f"Failed to connect to Meshtastic: {e}")
            raise

    def _on_packet_received(self, packet, interface):
        """Внутренний обработчик всех пакетов из Mesh."""
        try:
            sender_id = packet.get("fromId")
            decoded = packet.get("decoded", {})
            portnum = decoded.get("portnum")
            
            # 1. Обработка телеметрии (CUSTOM_APP порта или POSITION_APP)
            if portnum == "TELEMETRY_APP" or portnum == "POSITION_APP":
                self._handle_telemetry(sender_id, decoded, packet)
            
            # 2. Обработка текстовых сообщений (команд)
            elif portnum == "TEXT_MESSAGE_APP":
                payload = decoded.get("text", "")
                self._handle_command(sender_id, payload)
                
        except Exception as e:
            logger.error(f"Error processing packet: {e}")

    def _handle_telemetry(self, node_id: str, data: Dict, packet: Dict):
        """Обновляет состояние соседа и уведомляет подписчиков."""
        telemetry = {
            "node_id": node_id,
            "timestamp": time.time(),
            "snr": packet.get("rxSnr"),
            "hop_limit": packet.get("hopLimit"),
            "data": data
        }
        self.neighbors[node_id] = telemetry
        
        for callback in self.on_telemetry_received:
            callback(telemetry)

    def _handle_command(self, sender_id: str, payload: str):
        """Проверка и выполнение входящей команды."""
        logger.info(f"Received command from {sender_id}: {payload}")
        try:
            # Пробуем распарсить JSON команду
            cmd_data = json.loads(payload)
            for callback in self.on_command_received:
                callback(sender_id, cmd_data)
        except json.JSONDecodeError:
            # Если не JSON, передаем как строку
            for callback in self.on_command_received:
                callback(sender_id, {"raw": payload})

    def broadcast_telemetry(self, telemetry_data: Dict):
        """Рассылка собственной телеметрии всем участникам роя."""
        if not self.interface:
            return
        
        payload = json.dumps(telemetry_data)
        self.interface.sendText(payload, wantAck=False)
        logger.debug(f"Broadcasted local telemetry: {payload}")

    def send_command(self, target_node: str, command: Dict):
        """Отправка адресной команды конкретному узлу."""
        if not self.interface:
            return
        
        payload = json.dumps(command)
        self.interface.sendText(payload, destinationId=target_node, wantAck=True)
        logger.info(f"Sent command to {target_node}: {payload}")

    def get_neighbor_states(self) -> List[Dict]:
        """Возвращает список актуальных состояний всех соседей."""
        # Можно добавить фильтрацию по времени (удалять тех, кто пропал > 5 мин назад)
        return list(self.neighbors.values())

    def close(self):
        if self.interface:
            self.interface.close()