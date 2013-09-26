# edrone AR drone control library for Erlang

edrone_client.erl implements the standard commands
used to control the standard AR drone 2.0 (may work with
version 1.0 as well)

In priv directory there is also an Erlang that can be
installed on the AR drone 2.0 (not 1.0) it self.

## Install Erlang procedure

After attaching the wifi network announced by the drone,
start with transfer the erlang archive and the libutil file to the drone, 
any user (like root) and no password is passed to ftp.

    ftp 192.168.1.1
    >put arm-unknown-linux-gnueabi-erl.tgz
    >put libutil-2.13.so
	
Then telnet to the target

    telnet 192.168.1.1

You will find the archive placed in /data/video, now
unpack and install erlang

    cd /date/video
    tar xzf arm-unknown-linux-gnueabi-erl.tgz
    rm  arm-unknown-linux-gnueabi-erl.tgz
    mv /data/video/arm-unknown-linux-gnueabi-erl /data/erlang

and the shared object libutil-2.13.so.

    mv /data/video/libutil-2.13.so /lib
    cd /lib
    ln -s libutil-2.13.so libutil.so.1

Install and check for a prompt

    cd /data/erlang 
    ./Install -minimal /data/erlang
    /data/erlang/bin/erl


Copy the additional erlang libraries to the drone:
      sh transfer-libs.sh  

This command will ftp edrone.tgz, gpio.tgz, i2c.tgz and uart.tgz to the edrone (192.168.1.1),
and then install these packages under /data/erlang/lib

## Loading code with distributed erlang

Instead of transfering libraries and files to the drone every
time a beam files is changed the erl_prim_loader can be
told to told from the development machin instead.

Erlang on the drone is then started like:

    erl -loader inet -id drone -hosts 192.168.1.2 

To make the drone node a distribute node then give the following arguments
    
    -sname drone -setcookie <cookie>

On the development a node with a bootserver must have been started

    erl -kernel start_boot_server true boot_server_slaves '[{192,168,1,1}]'

The boot server node may also be a distributed node by giving the following argument to the command line:

    -sname devnode -setcookie <cookie>


Note the argument to boot_server_slaves, for some reason the manual is wrong
here and this is the only accepted format.
<cookie> is any string, like a password.
This erlang system will list on a tcp port number 555 and and udp
port 546.
