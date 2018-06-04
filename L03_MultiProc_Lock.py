# L03_MultiProc_Lock.py
# https://www.journaldev.com/15631/python-multiprocessing-example
# https://docs.python.org/3.6/library/multiprocessing.html

'''
Python multiprocessing Lock Class
The task of Lock class is quite simple. It allows code to claim lock
so that no other process can execute the similar code until the lock
has be released.

So the task of Lock class is mainly two. One is to claim lock and
other is to release the lock.
... To claim lock the, acquire() function is used
... To release lock release() function is used.

In the code below
=================
Suppose we have some tasks to accomplish.
To get that task done, we will use several processes.

So, we will maintain two queue.
One will contain the tasks and the other
will contain the log of completed task.

Then we instantiate the processes to complete the task.
Note that the python Queue class is already synchronized.
                     ===========    ====================
i.e. we don’t need to use the Lock class to block multiple
process to access the same queue object.
That’s why, we don’t need to use Lock class in this case.

Below is the implementation where
... we are adding tasks to the queue,
... then creating processes and starting them,
... then using join() to complete the processes.
... Finally we are printing the log from the second queue

NB: there are MORE tasks than processes

'''

from multiprocessing import Lock, Process, Queue, current_process
import time
import queue # imported for using queue.Empty exception


def do_job(tasks_to_accomplish, tasks_that_are_done):
    # each multiproc Process will execute the following code, if there are 4, then 4 will them will do so.
    # they will each try to obtain a task infinitely until except 'queue.Empty i.e. no task left' is raised.
    while True:     # loop forever until 'break' occurs            
        try:
            '''
                try to get task from the queue.
                .get_nowait() function will raise queue Empty exception if the queue is empty. 
                queue(False) function would do the same task also.
                
            '''
            task = tasks_to_accomplish.get_nowait()
        except queue.Empty:
            # queue is empty, break out of infinite loop
            break
        else:
            '''
                if no exception has been raised, add the task completion message
                to task_that_are_done queue
                
            '''
            print(task)
            tasks_that_are_done.put(task + ' is done by ' + current_process().name)
            time.sleep(.5)

    return True


def main():
    number_of_task = 10    
    number_of_processes = 4    
    
    tasks_to_accomplish = Queue()
    tasks_that_are_done = Queue()
    processes = []

    # put tasks in tasks_to_accomplish's queue
    for i in range(number_of_task):
        tasks_to_accomplish.put("Task no " + str(i))

    # creating processes
    for w in range(number_of_processes):
        p = Process(target=do_job, args=(tasks_to_accomplish, tasks_that_are_done))
        processes.append(p)
        p.start()

    # completing process
    for p in processes:
        p.join()

    # print the output
    while not tasks_that_are_done.empty():
        print(tasks_that_are_done.get())

    return True


if __name__ == '__main__':
    main()
