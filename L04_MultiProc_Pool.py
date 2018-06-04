# L04_MultiProc_Pool.py
# https://www.journaldev.com/15631/python-multiprocessing-example
# https://docs.python.org/3.6/library/multiprocessing.html

'''
Python multiprocessing Pool can be used for parallel execution of a function across multiple input values,
distributing the input data across processes (data parallelism).

Below is a simple Python multiprocessing Pool example.

'''

from multiprocessing import Pool
import time

work = (["A", 5], ["B", 2], ["C", 1], ["D", 3])


def work_log(work_data):
    print(" Process %s waiting %s seconds" % (work_data[0], work_data[1]))
    time.sleep(int(work_data[1]))
    print(" Process %s Finished." % work_data[0])


def pool_handler():
    # for this prg, pool size is 2, so two executions of work_log function is
    # happening in parallel. When one of the function processing finishes,
    # it picks the next argument and so on.
    p = Pool(2)             
    p.map(work_log, work)


if __name__ == '__main__':
    pool_handler()
