edrone AR drone control library for Erlang
==========================================

edrone_client.erl implements the standard commands
used to control the standard AR drone 2.0 (may work with
version 1.0 as well)

In priv directory there is also an Erlang that can be
installed on the AR drone 2.0 (not 1.0) it self.

# Install Erlang procedure

Start with transfer the archive to the drone, use root as user
and no password.

    ftp 192.168.1.1
    >put arm-unknown-linux-gnueabi-erl.tgz
	>put libutil-2.13.so
	
Then telnet to the target

    telnet 192.168.1.1

You will find the archive placed in /data/video, now
unpack and install erlang

    cd /date/video
    tar xf arm-unknown-linux-gnueabi-erl.tgz
    rm  arm-unknown-linux-gnueabi-erl.tgz
    mv /data/video/arm-unknown-linux-gnueabi-erl /data/erlang

and the shared object libutil-2.13.so.

    mv /data/libutil-2.13.so /lib
	cd /lib
	ln -s libutil-2.13.so libutil.so.1

Install and check for a prompt

    cd /data/erlang 
    ./Install -minimal /data/erlang
    /data/erlang/bin/erl
