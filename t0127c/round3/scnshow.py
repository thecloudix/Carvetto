import sys
for l in open(sys.argv[1]):
    t=l.split()
    f=lambda x: '%.3f'%float.fromhex(x) if x.startswith(('0x','-0x')) else x
    if t[0] in 'CLAB': print(t[0],' '.join(f(x) for x in t[1:]))
    elif t[0]=='P': v=[f(x) for x in t[1:]]; print('P D',v[0],'vdeg',v[1],'allow',v[2],'side',v[3],'dir',v[4],'start',v[5],'cut',v[6],'pass',v[7],'lpt',v[8],'sep',v[10],'lpa',v[11],'rev',v[12],'tabs',v[13],'len',v[14],'th',v[15],'spacing',v[16],'count',v[17],'dist',v[18],'avoid',v[20],'3d',v[21])
