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



# The flying bit

Connect a Logitech Extreme 3D Pro joystick to a PC connected to the AR Drone WiFi access point.


1. On the PC, start erlang and fire up the edrone_pilot app:

   ```
   erl 
   application:start(inpevt).
   application:start(edrone_pilot).
   ```

2. Telnet to the AR Drone

   ```
   telnet 192.168.1.1
   ```

3. Kill the original AR Drone flight control software.

   ```
   ps | grep program.elf | head -n 1 - | cut -c 1-5 | xargs kill
   ```

4. Start an erlang prompt on the Drone:

    ```
    /data/erlang/bin/erl
    ```

5. Start the necessary apps on the drone
    On the erlang command prompt, enter:

    ```
    application:start(uart).
    application:start(gpio). 
    application:start(edrone). 
    edrone_control:flat_trim(). 
    edrone_control:enable().
    ```
    
## Joystick controls

- Stick forward<br>
  Dip the nose of the drone, making it move forward.

- Stick backward<br>
  Raise the nose of the drone, making it move backward.

- Stick right<br>
  Raise the left side of the drone, making it move to its left.

- Stick left<br>
  Raise the right side of the drone, making it move to its right.

- Rotate stick left<br>
  Make the drone rotate anti clockwise.

- Rotate stick right<br>
  Make the drone rotate clockwise.

- Throttle (base of the joystick)<br>
  Throttles the motors between off and max.

- Fire button<br>
  Press and hold to disable throttle control and lock altitude at the current level.

- Thumb button (left side of stick)<br>
  Activate a newly uploaded edrone_control.beam file (through edrone_upgrade.erl).


# Demo script (to be simplified)

1. Edit the edrone_control.erl file to introduce glitch<br>
Set the HAVE_GLITCH value in edrone_control.erl, line 58, to 0.

2. Make sure that the development PC is connected to AR Drone WiFi network<br>
To avoid interferrence with wired networks that uses the 192.168.1 subnet, disconnect any
ethernet cable from the PC.

3. Compile and upload new file<br>
   Move to edrone home directory, and run the following commands

   ```
   rebar compile
   cd priv
   sh transfer-libs.sh
   ```

   The new software image, with a glitch, is now installed on the drone.

4. Start drone pilot software on the development PC<br>
   See previous chapter for start instructions.

5. Start drone software on AR Drone<br>
   See previous chapter for start instructions. 
   Ensure that the Drone is located in an open area, with its nose (camera) pointing 
   away from you. The glitch may cause temporary, low-level operaiton of the motors
   while the drone is on the ground.

6. Remove the glitch fromn edrone_control.erl<br>
   Set the HAVE_GLITCH value in edrone_control.erl, line 58, to -1.

7. Recompile and transfer the fixed software to the drone<br>
   Repeat step 3 above to compile and transfer the image.

8. Raise the drone to 1 meters height<br>
   Apply throttle until the desired height
   is reached. Control pitch, roll and yaw roll using the the
   joystick. The glitch will cut out the motors at irregular
   intervals, but the flight control system will compensate.

9. Press and hold fire button to lock altitude<br>
   Throttle will be deactivated for as long that you hold the button. The locked 
   altitude will make it easier to see the glitch and its effects on the drone's
   flight
   

10. Press the thumb button to upgrade software<br>
    The glitch will dissappear and flight will be stabilized.

11. Release the fire button to disengage altitude lock<br>

12. Land the drone<br>
    Throttle back until the drone has landed.

