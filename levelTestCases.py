'''
This file and it's content is based upon the project AI-Multi-Agent-Systems-in-Sokoban
by Diego Tobarra
Git project: https://github.com/akkerd/AI-Multi-Agent-Systems-in-Sokoban
'''


import unittest

#from os import path
import subprocess
import shlex, re


class SolveLevelTestCase(unittest.TestCase):
    """
    Application tests for solving levels.
    """
    # TODO: we could add options if we want to.
    # TODO: fix in the case where no solution is found
    def callServer(self,level='SAD1.lvl', strategy='bfs', ma=False):

        if ma:
            client = 'main.py'
        else:
            client = 'searchclient.py'

        args = str(r'java -jar server.jar -l ' + level
                   + ' -c "python3 main.py -s ' + strategy + '"')

        args = str(r'java -jar server.jar -l ' + level
                   + ' -c "python3 main.py "')
        argsarr = shlex.split(args)
        solution = {}
        with subprocess.Popen(argsarr, stderr=subprocess.PIPE) as proc:
           while True:
                line = str(proc.stderr.readline())

                solutionfoundregex = re.match("^.*Found solution of length (?P<length>\d+)\..*$",line)
                failregex = re.match(".*(Exception|Error).*",line)
                if solutionfoundregex:
                    solution['length'] = int(solutionfoundregex.group('length'))
                    line = str(proc.stdout.readline())
                    statisticsregex = re.match('^.*#Explored:\s*(?P<explored>\d+).*#Frontier:\s*(?P<frontier>\d+).*Time:\s*(?P<time>\d+\.?\d*)\s*s,.*Alloc:\s*(?P<alloc>\d+\.?\d*)\s*MB,\s*MaxAlloc:\s*(?P<maxalloc>\d+\.?\d*)\s*MB.*\..*$',line)
                    if statisticsregex:
                        solution['explored'] = int(statisticsregex.group('explored'))
                        solution['frontier'] = int(statisticsregex.group('frontier'))
                        solution['time'] = float(statisticsregex.group('time'))
                        solution['alloc'] = float(statisticsregex.group('alloc'))
                        solution['maxalloc'] = float(statisticsregex.group('maxalloc'))
                        proc.stderr.close()
                        proc.kill()
                        #return [agent1.act() for _ in range(0,len(agent1.solution))]
                        return solution
                elif failregex:
                    proc.stderr.close()
                    proc.kill()
                    raise Exception('Error found')

    def setUp(self):
        unittest.TestCase.setUp(self)

    def tearDown(self):
        unittest.TestCase.tearDown(self)


class SolveSALevelTestCase(SolveLevelTestCase):
    def testSolveSAD1(self):
        solution = self.callServer(level='./levels/SAD1.lvl', strategy='bfs')
        self.assertEqual(solution['length'], 19)

    def testSolveSAsoko3_24(self):
        solution = self.callServer(level='./levels/SAsoko3_24.lvl', strategy='greedy')
        self.assertEqual(solution['length'], 440)

    def testSolveSAsimple2(self):
        solution = self.callServer(level='./levels/SAsimple2.lvl', strategy='dfs')
        self.assertEqual(solution['length'], 516)


class SolveMALevelTestCase(SolveLevelTestCase):
    def testSolveMASimple(self):
        solution = self.callServer(level='./levels/SAtestagent.lvl', ma=True)
        self.assertEqual(7, solution['length'])



if __name__ == '__main__':
    unittest.main()