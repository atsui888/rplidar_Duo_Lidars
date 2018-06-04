# Listing 1 MultiProc_Args.py
# https://www.journaldev.com/15631/python-multiprocessing-example
# https://docs.python.org/3.6/library/multiprocessing.html

'''
    Process class has two important functions
    .start() and .join()

    steps:
    . write a fn that will be run by the process
    . instantiate a process object
    . nothing will happen until we tell it to start processing via .start() fn
    . then the process will run and return its result.
    . after that we tell the process to complete via .join()

    . without .join(), the process will remain idle and won't terminate

    . to pass any argument through the process you need to use args keyword argument.


'''

from multiprocessing import Process


def print_func(continent='Asia'):
    # this fn will be run by multiprocessing.
    print('The name of continent is : ', continent)

if __name__ == "__main__":  # confirms that the code is under main function
    names = ['America', 'Europe', 'Africa']
    procs = []


    # MultiProcessing Instantiating ... without arguments
    for name in names:
        proc = Process(target=print_func)   # instantiating without any argument
        procs.append(proc)                  # add latest proc to list procs        
        proc.start()                        # start the multiprocessing proc


    # instantiating process with arguments
    # nMultiProcessing Instantiating ... without arguments
    for name in names:
        # print(name)                       # for debugging
        proc = Process(target=print_func, args=(name,))
        procs.append(proc)     
        proc.start()

    # complete the processes - processes join back the main thread
    for proc in procs:
        proc.join()




