import cPickle

import sys
if '..' not in sys.path:
    sys.path.append('..')
#=======================================================================
# need lupa
# https://pypi.python.org/pypi/lupa
#=======================================================================

import lupa
from lupa import LuaRuntime

lua = LuaRuntime(unpack_returned_tuples=True)

def luaTableToDict(luaTable):
    result = dict()
    for i in luaTable:
        result[i] = luaTable[i]
        if lupa.lua_type(result[i]) is 'table':
            result[i] = luaTableToDict(result[i])
    return result

def readLuaTableFile(luaTableFilePath):
    luaTable = lua.eval('dofile("'+luaTableFilePath+'")')
    result = luaTableToDict(luaTable)
    return result

def saveDict(dictFilePath, dictData):
    dictfile = open(dictFilePath, 'wb')
    cPickle.dump(dictData, dictfile)
    dictfile.close()

def saveDictFromLuaTableFile(dictFilePath, luaTableFilePath):
    saveDict(dictFilePath, readLuaTableFile(luaTableFilePath))

#======================================================================
# test
#======================================================================

def testReadLuaTable():
    path = "/home/jo/Research/yslee/Resource/motion/opensim/FullBody2_lee.luamscl"
    result = readLuaTableFile(path)
    for i in result:
        print i, result[i]
    for i in result[1]:
        print i, result[1][i]

def testReadAndWrite():
    lua_path = "/home/jo/Research/yslee/Resource/motion/opensim/FullBody2_lee.luamscl"
    dict_path = "dictResult"
    saveDictFromLuaTableFile(dict_path, lua_path)

def testWrittenFile():
    dict_path = "dictResult"
    dictfile = open(dict_path, 'rb')
    result = cPickle.load(dictfile)
    dictfile.close()
    for i in result:
        print i, result[i]
    for i in result[1]:
        print i, result[1][i]


if __name__=='__main__':
    testReadLuaTable()
    testReadAndWrite()
    testWrittenFile()
