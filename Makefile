obj-m += fjbtndrv.o

fjbtndrv-objs := fjbuttons.o

#ifneq ($(__KERNEL__),)
#endif


default:
	@make -C /lib/modules/`uname -r`/build M=`pwd` modules

install:
	@make -C /lib/modules/`uname -r`/build M=`pwd` modules_install

clean:
	@make -C /lib/modules/`uname -r`/build M=`pwd` clean
