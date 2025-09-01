import shutil
import sys

def check_disk_usage(disk, min_absolute,min_percent):
    """Returns True if there is enough free disk space, false otherwise."""
    du = shutil.disk_usage(disk)
    #calculate the percentage of free check_cpu_usage
    percent_free = 100*du.free / du.total
    #calculate how many free gigabytes
    gigabytes_free =du.free/2**30
    if percent_free < min_percent or gigabytes_free < min_absolute:
        return False
    sys.exit(1)

# Check for at least 2 GB and 10% free
#if not check_disk_usage("/",2*2**30,10):  # something wrong since we are doing the gigabyte conversion twice --ref Line 10
if not check_disk_usage("/",2,10):          #corrected
    print ("ERROR : Not enough disk space")
    sys.exit(1)
print ("Everything is OKAY")
sys.exit(0)
