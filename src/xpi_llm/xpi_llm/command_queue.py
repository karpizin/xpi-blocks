"""One worker thread per node for LLM commands.

Starting a thread per command let two commands run at the same time and finish
in any order (a "stop" could be overtaken by the "go" before it). Commands are
now handled one at a time, in arrival order; when too many are waiting, new
ones are refused instead of piling up behind slow LLM calls.
"""
import queue
import threading

_STOP = object()


class CommandQueue:
    def __init__(self, handler, maxsize=4, on_error=None):
        self._handler = handler
        self._on_error = on_error
        self._queue = queue.Queue(maxsize=maxsize)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, item):
        """Queue ``item``; False if the queue is full (the command is dropped)."""
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            return False

    def _run(self):
        while True:
            item = self._queue.get()
            try:
                if item is _STOP:
                    return
                self._handler(item)
            except Exception as e:  # noqa: BLE001 - one bad command must not kill the worker
                if self._on_error:
                    self._on_error(e)
            finally:
                self._queue.task_done()

    def join(self):
        """Wait until every queued command has been handled (for tests)."""
        self._queue.join()

    def stop(self, timeout=1.0):
        try:
            self._queue.put_nowait(_STOP)
        except queue.Full:
            pass
        self._thread.join(timeout)
