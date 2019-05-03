from pynput.keyboard import Key, Listener
import Queue
import threading
import sys
class KeyboardListener(object):

    def __init__(self):
        print("keyboard listener is on")
        self.actionqueue = Queue.Queue(maxsize=1)
        self._listener = threading.Thread(target=self._listener_thread)
        self._listener.start()

    def _on_press(self, key):
        # print('{0} pressed'.format(key))
        try:
            if key == 'z':
                self.queue_key(key)
                return False
            print('{0} pressed'.format(key))
            self.queue_key(key)
        except AttributeError:
            pass
        
    def _on_release(self, key):
        try:
            #print('{0} released'.format(key))
            if key == 'z':
                self.queue_key(key)
                return False
            self.queue_key(key)
            
        except AttributeError:
            pass

    # Collect events until released
    def get_button(self):
        if not self.actionqueue.empty():
            return self.actionqueue.get()
        else:
            return None

    def _listener_thread(self):
        with Listener(
                on_press=self._on_press) as listener:
                listener.join()
                sys.exit(0)

    def queue_key(self, key):
        if self.actionqueue.full():
            self.actionqueue.get()
        self.actionqueue.put(key)

if __name__=="__main__":
    keyboard = KeyboardListener()
    while True:
        print keyboard.get_button()