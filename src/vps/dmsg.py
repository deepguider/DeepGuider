def dmsg(text_s = '', verbose = False): # Usage : dmsg('any message')
    if verbose:
        return
    import inspect
    frame = inspect.currentframe()
    fname = str.split(str(frame.f_back.f_code),'"')[1] # <code object dmsg at 0x7f63ad0a08a0, file "./../src/vps/vps.py", line 47>
    line = str(frame.f_back.f_lineno)
    print ('dmsg() at ' + fname + ' : ' + line + ' ' + text_s)
    return
