

import threading


def set_all_threads(*threads:threading.Thread, set_threads_on:bool=True):
    if set_threads_on:
        for thread in threads:
            thread.start()
    else:
        for thread in threads:
            thread.join()
            del thread



def set_all_events(*events:threading.Event, set_events_on :bool= True):
    if set_all_events:
        for event in events:
            event.set()
    else:
        for event in events:
            event.clear()

