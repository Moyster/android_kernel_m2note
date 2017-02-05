[M571](http://www.meizu.com)
=================

Linux kernel source code for the Meizu M2 Note.

HOW TO COMPILE
-----------

###1. Download source code###

  <code>git clone https://github.com/Moyster/android_kernel_m2note.git</code>

###2. Compiling###

Make sure your device tree uses :  
```
TARGET_KERNEL_CONFIG := los14_m2note_defconfig
```
PS: this branch is only for Nougat, see branch "master" for LP/MM support  

From within the Android Sources you want to build, do :  

```
. build/envsetup.sh && breakfast m2note  
make -jX bootimage  
(where -jX is your number of cores + 1, -j5 for a quadcore)  
```

This will give you a flashable boot.img located in out/target/product/device-xyz/  

