# L02_MultiProc_Queue.py
# https://www.journaldev.com/15631/python-multiprocessing-example
# https://docs.python.org/3.6/library/multiprocessing.html

'''
Python Multiprocessing modules provides Queue class that is exactly a
First-In-First-Out data structure.

They can store any pickle Python object (though simple ones are best) and
are extremely useful for sharing data between processes.

Queues are specially useful when passed as a parameter to a Process’ target function
to enable the Process to consume data. By using put() function we can insert data to
then queue and using get() we can get items from queues.

'''

from multiprocessing import Queue

colors = ['red', 'green', 'blue', 'black']
cnt = 1

# instantiating a queue object 
queue = Queue()

# and push items into queue, .put(item)
print('pushing items to queue:')
for color in colors:
    print('item no: ', cnt, ' ', color)
    queue.put(color)
    cnt += 1

# and pop items into queue, .get(item)
# popping is FIFO, what goes in first is what comes out first
print('\npopping items from queue:')
cnt = 0
while not queue.empty():
    print('item no: ', cnt, ' ', queue.get())
    cnt += 1
