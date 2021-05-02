#!/usr/bin/env python
import subprocess, shlex
import socket, sys, time, datetime, signal, os, re
from shutil import copyfile
from shutil import copy
import logger as log
#import sdfmutator as sdf
#import output_parser 
from os.path import expanduser
import psutil
from termcolor import colored, cprint
import random
from tokenize import tokenize
import pickle

GAZEBO = 'gzserver'

GENETIC_HOME = expanduser("~") + "/genetic"
AUTO_HOME = expanduser("~") + "/ardupilot/Tools/autotest"
script_map = None
M = 3
V = 5


def kill_tasks():
    pname = ["gzserver", "arducopter", "APMrover2.elf", "MAVProxy", "sim_vehicle", "mavproxy", "xterm", "gazebo", "px4", "px4_iris"]
    #pname = ["gzserver", "MAVProxy", "sim_vehicle", "mavproxy", "gazebo"]

    p = subprocess.Popen(['ps', '-A'], stdout=subprocess.PIPE)
    out, err = p.communicate()
    for line in out.splitlines():
        if any(x in line for x in pname):
            pid = int(line.split(None, 1)[0])
            try:
                os.kill(pid, signal.SIGTERM)
                print "killing ", line
                time.sleep(0.1)
            except OSError:
                print "already killed ", pid, line

def runSim(id, cmd, config, TIMEOUT, MAINNAME, INIT_TIME, real_mode):
    '''
    run simulation (dummy or real sim)
    '''
    if (real_mode == True):      #real simulation

        # configure input and run simulation
        startSim(id, cmd, config, TIMEOUT, MAINNAME, INIT_TIME)

        # collect output
        output_vector = post_processing(id, config, MAINNAME)

    else:                   #dummy simulation
        log.cinfo("#%s Dummy Simulation running ..." % (id))

        # configure input and run dummy simulation
        replay_output = random.randint(0,1) 
        dummy_output = [random.randint(0,V), random.random(), random.randint(0,V)] # R(.) [F1 F2 F3]
        sim_outfile_name = 'simout_' + id + '.log'
        sim_outfile = GENETIC_HOME + '/outputs/' + sim_outfile_name
        if not os.path.exists(GENETIC_HOME + '/outputs'):   
            os.makedirs(GENETIC_HOME + '/outputs')
        outf = open(sim_outfile, "w")
        outf.write(str(replay_output)+' ')
        outf.write(' '.join(map(str,dummy_output)))
        outf.write('\n')
        outf.close()

        # collect output
        sim_outfile = GENETIC_HOME + '/outputs/' + sim_outfile_name
        print sim_outfile
        with open(sim_outfile, 'r') as current:
            line = current.readline()
        tokenizer = re.compile('\s+')
        tokens = tokenizer.split(line)
        output_vector = [tokens[0], tokens[1:4]]     #output_vector:[code, [objectives]]
        print "[output_vector] ", output_vector
        log.cinfo("=== Collecting outputs (dummy)")

    return output_vector

'''
run simulaiton 
output: simulation output file (simulation results: R(.) and objectives) e.g., 1 X X X
'''
def startSim(id, cmd, config, TIMEOUT, MAINNAME, INIT_TIME):

    isNormal = True
    start_time = datetime.datetime.now()

    log.cinfo("#%s Simulation running ..." % (id))
    print "start: ", start_time

    # display pid
    #print "subprocess pid: ", p.pid                     #main process ('/bin/sh', cd /home...)
    #main_process = psutil.Process(p.pid)
    #children = main_process.children(recursive=True)    #child process ('python', ./sim_vheicle.py...)
    #for child in children:
    #    print 'child pid: ', child.pid

    # system-specific initialization
    if (MAINNAME == 'arducopter'):
        global script_map
        # param, old_value, new_value, time
        script_map = [
            ['ATC_RAT_RLL_P',       0.091,      1,      17], \
            ['ATC_THR_MIX_MIN',     0.1,        0.25,   0], \
            ['RTL_ALT',             4500,       1500,   0], \
            ['PSC_ANGLE_MAX',       0,          45,     0], \
            ['PILOT_TKOFF_ALT',     214,        200,    0] \
        ]
        config_script_file = AUTOHOME + '/config_params.scr'
        configf = open(config_script_file, "w")

        # restore to old value before start
        for i in range(len(script_map)):
            configf.write('param set %s %s \n' % (script_map[i][0], script_map[i][1]))
            # configf.write(f'param set {script_map[i][0]} {script_map[i][1]} \n')

        # use config to generate input setup 
        
        # define your global param mape for each system
        print "config:", config

        online_cfg = []
        for i in range(len(config)):
            if config[i] == 1:
                if script_map[i][3] == 0:
                    configf.write('param set %s %s \n' % (script_map[i][0], script_map[i][2]))
                    # configf.write('param set ' + script_map[i][0] + str(script_map[i][2]))
                else:
                    online_cfg.append(['param set %s %s \n' % (script_map[i][0], script_map[i][2]), script_map[i][3]])
                    # online_cfg.append(script_map[i])
        configf.write('wp load pu_square_mission.txt\n')
        configf.close()

        print "write online config pickle"
        online_cfg_file = AUTOHOME + '/online_cfg.p'   
        pickle.dump(online_cfg, open(online_cfg_file, "wb" ))
        
        
    
    # start process
    # p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    runtime_log_file = GENETIC_HOME + '/outputs/shellout_' + id + '.txt'
    # open(runtime_log_file, 'w')
    p = subprocess.Popen(cmd, shell=True, stdout=open(runtime_log_file, 'w'), stderr=subprocess.STDOUT)
    time.sleep(1.0)

    while True: 
        now = datetime.datetime.now()
        print ' Waiting for Initialization for', INIT_TIME, '.. ', (int)(INIT_TIME - (now-start_time).seconds), '\r',
        sys.stdout.flush()
        if (datetime.datetime.now() - start_time).seconds >= INIT_TIME:
            break
        time.sleep(0.1)
    print "\nRunning..."

    #time.sleep(20)  # wait for initiating Gazebo, main...

    #for proc in psutil.process_iter():
    #    #print '\t\t', proc.pid, proc.name(), proc.cmdline()
    #    if proc.pid == p.pid:
    #        print ("main [%d %s %s]" % (proc.pid, proc.name(), proc.cmdline()))
    #    for child in children:
    #        if proc.pid == child.pid:
    #            print ("child [%d %s %s]" % (proc.pid, proc.name(), proc.cmdline()))


    PROCNAME = MAINNAME
    elf_pid = None
    gazebo_pid = None

    if (MAINNAME == 'arducopter'):

        # check finish
        while p.poll() is None:

            time.sleep(1)
            # progress
            now = datetime.datetime.now()
            progress_sec = (now-start_time).seconds #elapsed second
            progress = int((progress_sec*100 / TIMEOUT))    # (sec)
            print ' [ ' + str(progress) + "%]", (int)(TIMEOUT - progress_sec), "sec to timeout\r",
            sys.stdout.flush()
            #print (' [%3d] Running... (%d sec to timeout)'  % (progress, (int)(TIMEOUT-progress_sec)))
            #sys.stdout.flush()

	    	## check main alive
            # get pid for elf
            if elf_pid == None:
                plist = subprocess.Popen(['ps', '-A'], stdout=subprocess.PIPE)
                out, err = plist.communicate()
                #print out  #list all 
                for line in out.splitlines():
                    elf_pid = None
                    if MAINNAME in line:
                        #print "Main Program Info :", line
                        if not 'defunct' in line:
                            elf_pid = int(line.split(None, 1)[0])
                            break
                if elf_pid != None:
                    print "Main Program : ", MAINNAME, elf_pid

            # check not exist => kill all        
            if psutil.pid_exists(elf_pid) != True: 
                    # or psutil.Process(elf_pid).status() == psutil.STATUS_ZOMBIE:
                cprint("\n System Info: ", 'yellow')
                print " [", MAINNAME, elf_pid, "] is not alive "
                time.sleep(3)
                kill_tasks()
                break
            else:
                print "\n", elf_pid, MAINNAME, " alive"

            #check timeout
            if (now - start_time).seconds >= TIMEOUT:
                print "\nTIMEOUT kill process ", p.pid
                os.kill(p.pid, signal.SIGTERM)
                os.waitpid(-1, os.WNOHANG)
                kill_tasks()
                isNormal = False
        #while end


    end_time = datetime.datetime.now()
    print "end: ", end_time
    print "elaped: ", end_time - start_time
    kill_tasks()
    log.info("\n  --- Simulation %s completed ---\n" % (id,))

def post_processing(simlog_id, config, system_name):

    if(system_name == 'arducopter'):
        # Simulation (raw BIN) log
        last_sim_log = AUTOHOME + '/logs/LASTLOG.TXT'
        if not os.path.isfile(last_sim_log):
            print "[ERROR]: Simulation was not exectued properly -- no LASTLOG.TXT file", last_sim_log
            exit()

        sim_log_name = open(last_sim_log).readline().rstrip().rjust(8,'0') + ".BIN"
        sim_log_file = AUTOHOME + '/logs/' + sim_log_name
        if not os.path.isfile(sim_log_file):
            print "[ERROR]: Simulation was not exectued properly -- no BIN log file", sim_log_file
            exit()

        if not os.path.exists(GENETIC_HOME + '/outputs'):   
            os.makedirs(GENETIC_HOME + '/outputs')

        print("simlog(BIN) copy: " + sim_log_file + ' --> ' + GENETIC_HOME + '/outputs/simout_' + simlog_id + '.BIN')

        # copy BIN log to output folder
        if not os.path.isfile(sim_log_file):
            print("simlog(BIN) not exists: " + sim_log_file)
        else:
            copyfile(sim_log_file, GENETIC_HOME + '/outputs/simout_' + simlog_id + '.BIN')

        # -------------------------------------------------- 
        # TODO
        # simulation output (costs) from control program
        # sim_outfile_name = 'simout_' + simlog_id + '.log'       # the simulator should make this file for fitness 
        sim_outfile_name = 'simout.log' 
        sim_outfile = GENETIC_HOME + '/outputs/' + sim_outfile_name
        with open(sim_outfile) as f:
            all_lines = f.readlines()
            if len(all_lines) > 0:
                r = 1
            else:
                r = 0
        f1 = 0
        for x in config:
            if x == 0:
                f1 += 1
        output_vector = [r, [f1, 0, 0]]

        # output_vector: [replay(.), [cost1, cost2, ...]]
        # process the simulation output and make a output vector
        # output_vector = [1, [2, 0, 0]]
        # -------------------------------------------------- 

        
        print("[output_vector] " + str(output_vector))
        log.cinfo("=== Collecting outputs: %s %s" % (sim_log_file, sim_outfile))

    else:
        print "ERROR: Not supported system: ", system_name
        exit(0)
    
    print "------------------"
    exit()
    return output_vector


def main():
    global GENETIC_HOME, AUTOHOME, M, V

    GENETIC_HOME = expanduser("~") + "/genetic"
    AUTOHOME = expanduser("~") + "/ardupilot/Tools/autotest"
    M = 3   # the number objectives
    V = 5   # the number of configuration

    log_id = '000000_000001'    # log id 
    #COMMAND = "cd " + AUTOHOME + "; ./sim_vehicle.py -v ArduCopter -m --mav10 --map --console -I1 -D -K " + log_id 
    COMMAND = "cd " + AUTOHOME + "; ./sim_vehicle.py -v ArduCopter -L PU -f X  --map --console -m --RVplayer"  
    
    timeout = 100 + 60               # total allowed simulation time 
    mainname = 'arducopter'     # system name
    init_time = 20              # waiting time for initialization
    real_sim_mode = True;       # simulation model: True: real, False: dummy sim
    config = [1, 1, 1, 0, 1]

    for i in range(3):
        # start sim functions
        output_vector = runSim(log_id, COMMAND, config, timeout, mainname, init_time, real_sim_mode)
        print "#", i, "# ", output_vector


if __name__ == '__main__':
    main()
