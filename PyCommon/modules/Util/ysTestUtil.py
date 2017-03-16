if __name__ == '__main__':

    def template_printAndExec():
        str = '''
        a = [0,0,0,1,1,1,0,0,0]
        b = [1,1,0,1,1,0,0,0,0]
        a_append_b = a+b
        print a_append_b
        '''
        codes = str.split('\n')
        
        for code in codes:
            code = code.lstrip()
            if len(code)>0 and code[0] == '#':
                continue
            print code,
            if 'print' in code:
                print ':',
            exec code
            if 'print' not in code:
                print
            
    template_printAndExec()