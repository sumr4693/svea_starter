"""
    Generate a udev rule to rename the arduino
    Do NOT run this with any other arduino than the 
    one that should be used for pwm control connected to the computer.
"""

import subprocess, re, sys, argparse

parser = argparse.ArgumentParser(
    description='''Generate an udev rule to rename the arduino
Do NOT run this with any other arduino than the 
one that should be used for pwm control connected to the computer.''')

parser.add_argument('-d', '--dry', 
        help='Generate the .rules file in the current folder instead of in /etc/udev/rules.d',
        action='store_true')

parser.add_argument('-f', '--file-name', 
        default='pwm_arduino', 
        type=str,
        action='store',
        help='name of the .rules file'),

parser.add_argument('-p', '--device-path', 
        default='ttyACM0', 
        type=str,
        action='store',
        help='Current name of the device')

parser.add_argument('-n', '--device-name', 
        default='arduinoPWM', 
        type=str,
        action='store',
        help='After running this script the device should show up as /dev/[this argument]')

parser.add_argument('-o', '--override', 
        help='Override previous arduino name links',
        action='store_true')

parser.add_argument('-v', '--verbose', 
        help='Override previous arduino DEVLINKS',
        action='store_true')


args = parser.parse_args(sys.argv[1:])

# Check if a device with the chosen name already exists and is connected
if not args.override:
    try:
        sp_str = subprocess.check_output(["ls", "/dev/{}".format(args.device_name)], stderr=subprocess.STDOUT)
        if not args.dry:
            raise RuntimeError("Arduino with that name already connected, exiting without generating a new rule")
        else:
            print("Warning: Arduino with that name already connected")
    except subprocess.CalledProcessError:
        pass
dev_str = subprocess.check_output('udevadm info -q path -n /dev/{}'.format(args.device_path).split())
sp_str = subprocess.check_output(["udevadm", "info", "-ap", "/sys/class/tty/{}".format(args.device_path)])

serial_re = r'ATTRS\{serial\}=="[0-9A-Fa-f]+"'
vendor_re = r'ATTRS\{idVendor\}=="[0-9A-Fa-f]+"'
num_re = r'"[0-9A-Fa-f]+"'

serial_str = re.search(serial_re, sp_str).group()
serial_str = re.search(num_re, serial_str).group()

vendor_str = re.search(vendor_re, sp_str).group()
vendor_str = re.search(num_re, vendor_str).group()

if args.verbose:
    print("Serial number: {}".format(serial_str))
    print("Vendor Id: {}".format(vendor_str))

r_string = '''SUBSYSTEM=="tty"\\
, ATTRS{{serial}}=={}\\
, ATTRS{{idVendor}}=={}\\
, SYMLINK+="arduinoPWM"'''.format(serial_str, vendor_str)

if args.verbose:
    print("Rule:\n{}\n\n{}\n\n{}".format("#"*40, r_string, "#"*40))

if args.dry:
    fn = './99-{}.rules'.format(args.file_name)
else:
    fn = '/etc/udev/rules.d/99-{}.rules'.format(args.file_name)

with open(fn, 'w') as rules_file:
    rules_file.write(r_string)
if args.verbose:
    print('Rules file succesfully writen to "{}"'.format(fn))

if not args.dry:
    sp_str = subprocess.check_output('udevadm control --reload-rules'.split())
    sp_str = subprocess.check_output('udevadm test -a -p {}'.format(dev_str).split(), stderr=subprocess.STDOUT)
    if args.verbose:
        print("")
        print("Test run results:")
        print(sp_str)
    if not "creating link '/dev/arduinoPWM' to '/dev/ttyACM0'" in sp_str:
        raise RuntimeError("Link not created for some reason")



