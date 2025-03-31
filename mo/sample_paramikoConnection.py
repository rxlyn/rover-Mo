import paramiko

hostname = 'raspberry_pi_ip'
username = 'pi'
password = 'your_password'  # or use key-based auth

command = "echo drive > /tmp/motor_state.txt"  # Change 'drive' to the desired state

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(hostname, username=username, password=password)
stdin, stdout, stderr = client.exec_command(command)
print(stdout.read().decode(), stderr.read().decode())
client.close()
