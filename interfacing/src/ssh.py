#!/usr/bin/env python
import paramiko
import sys
import rospy

ip = '192.168.43.119'
user = 'starwberrycake'
pswd = 'rishab'

print("")
print("Connecting to %s at %s" %(user, ip))
print("")

ssh_client =paramiko.SSHClient()
ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

ssh_client.load_system_host_keys()
ssh_client.connect(ip, port = 22, username = user, password = pswd, look_for_keys = False)

print("Connected successfully")

stdin, stdout, stderr = ssh_client.exec_command(' ./a.out ')
print("DONE")
print(stderr.readline())  #print error
print(stdout.readline())  #print output
