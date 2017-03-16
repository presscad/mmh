import cPickle

def readPickle(filename):
    rfile = open(filename, 'rb')
    data = cPickle.load(rfile)
    rfile.close()
    return data

def writePickle(filename, data):
    wfile = open(filename, 'wb')
    cPickle.dump(data, wfile)
    wfile.close()
