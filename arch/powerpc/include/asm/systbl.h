/*
 * List of powerpc syscalls. For the meaning of the _SPU suffix see
 * arch/powerpc/platforms/cell/spu_callbacks.c
 */

SYSCALL(restart_syscall)	/* 0 */
SYSCALL(exit)
PPC_SYS(fork)
SYSCALL_SPU(read)
SYSCALL_SPU(write)
COMPAT_SYS_SPU(open)		/* 5 */
SYSCALL_SPU(close)
COMPAT_SYS_SPU(waitpid)
COMPAT_SYS_SPU(creat)
SYSCALL_SPU(link)
SYSCALL_SPU(unlink)			/* 10 */
COMPAT_SYS(execve)
SYSCALL_SPU(chdir)
COMPAT_SYS_SPU(time)
SYSCALL_SPU(mknod)
SYSCALL_SPU(chmod)			/* 15 */
SYSCALL_SPU(lchown)
SYSCALL(ni_syscall)
OLDSYS(stat)
SYSX_SPU(sys_lseek,ppc32_lseek,sys_lseek)
SYSCALL_SPU(getpid)			/* 20 */
COMPAT_SYS(mount)
SYSX(sys_ni_syscall,sys_oldumount,sys_oldumount)
SYSCALL_SPU(setuid)
SYSCALL_SPU(getuid)
COMPAT_SYS_SPU(stime)		/* 25 */
COMPAT_SYS(ptrace)
SYSCALL_SPU(alarm)
OLDSYS(fstat)
SYSCALL(pause)
COMPAT_SYS(utime)			/* 30 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
COMPAT_SYS_SPU(access)
COMPAT_SYS_SPU(nice)
SYSCALL(ni_syscall)			/* 35 */
SYSCALL_SPU(sync)
COMPAT_SYS_SPU(kill)
SYSCALL_SPU(rename)
COMPAT_SYS_SPU(mkdir)
SYSCALL_SPU(rmdir)			/* 40 */
SYSCALL_SPU(dup)
SYSCALL_SPU(pipe)
COMPAT_SYS_SPU(times)
SYSCALL(ni_syscall)
SYSCALL_SPU(brk)			/* 45 */
SYSCALL_SPU(setgid)
SYSCALL_SPU(getgid)
SYSCALL(signal)
SYSCALL_SPU(geteuid)
SYSCALL_SPU(getegid)		/* 50 */
SYSCALL(acct)
SYSCALL(umount)
SYSCALL(ni_syscall)
COMPAT_SYS_SPU(ioctl)
COMPAT_SYS_SPU(fcntl)		/* 55 */
SYSCALL(ni_syscall)
COMPAT_SYS_SPU(setpgid)
SYSCALL(ni_syscall)
SYSX(sys_ni_syscall,sys_olduname, sys_olduname)
COMPAT_SYS_SPU(umask)		/* 60 */
SYSCALL_SPU(chroot)
COMPAT_SYS(ustat)
SYSCALL_SPU(dup2)
SYSCALL_SPU(getppid)
SYSCALL_SPU(getpgrp)		/* 65 */
SYSCALL_SPU(setsid)
SYS32ONLY(sigaction)
SYSCALL_SPU(sgetmask)
COMPAT_SYS_SPU(ssetmask)
SYSCALL_SPU(setreuid)		/* 70 */
SYSCALL_SPU(setregid)
SYS32ONLY(sigsuspend)
COMPAT_SYS(sigpending)
COMPAT_SYS_SPU(sethostname)
COMPAT_SYS_SPU(setrlimit)	/* 75 */
COMPAT_SYS(old_getrlimit)
COMPAT_SYS_SPU(getrusage)
COMPAT_SYS_SPU(gettimeofday)
COMPAT_SYS_SPU(settimeofday)
COMPAT_SYS_SPU(getgroups)	/* 80 */
COMPAT_SYS_SPU(setgroups)
SYSX(sys_ni_syscall,sys_ni_syscall,ppc_select)
SYSCALL_SPU(symlink)
OLDSYS(lstat)
COMPAT_SYS_SPU(readlink)	/* 85 */
SYSCALL(uselib)
SYSCALL(swapon)
SYSCALL(reboot)
SYSX(sys_ni_syscall,compat_sys_old_readdir,sys_old_readdir)
SYSCALL_SPU(mmap)			/* 90 */
SYSCALL_SPU(munmap)
COMPAT_SYS_SPU(truncate)
COMPAT_SYS_SPU(ftruncate)
SYSCALL_SPU(fchmod)
SYSCALL_SPU(fchown)			/* 95 */
COMPAT_SYS_SPU(getpriority)
COMPAT_SYS_SPU(setpriority)
SYSCALL(ni_syscall)
COMPAT_SYS(statfs)
COMPAT_SYS(fstatfs)			/* 100 */
SYSCALL(ni_syscall)
COMPAT_SYS_SPU(socketcall)
COMPAT_SYS_SPU(syslog)
COMPAT_SYS_SPU(setitimer)
COMPAT_SYS_SPU(getitimer)	/* 105 */
COMPAT_SYS_SPU(newstat)
COMPAT_SYS_SPU(newlstat)
COMPAT_SYS_SPU(newfstat)
SYSX(sys_ni_syscall,sys_uname,sys_uname)
SYSCALL(ni_syscall)			/* 110 */
SYSCALL_SPU(vhangup)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
COMPAT_SYS_SPU(wait4)
SYSCALL(swapoff)			/* 115 */
COMPAT_SYS_SPU(sysinfo)
COMPAT_SYS(ipc)
SYSCALL_SPU(fsync)
SYS32ONLY(sigreturn)
PPC_SYS(clone)				/* 120 */
COMPAT_SYS_SPU(setdomainname)
SYSCALL_SPU(newuname)
SYSCALL(ni_syscall)
COMPAT_SYS_SPU(adjtimex)
SYSCALL_SPU(mprotect)		/* 125 */
SYSX(sys_ni_syscall,compat_sys_sigprocmask,sys_sigprocmask)
SYSCALL(ni_syscall)
SYSCALL(init_module)
SYSCALL(delete_module)
SYSCALL(ni_syscall)			/* 130 */
SYSCALL(quotactl)
COMPAT_SYS_SPU(getpgid)
SYSCALL_SPU(fchdir)
SYSCALL_SPU(bdflush)
COMPAT_SYS(sysfs)			/* 135 */
SYSX_SPU(ppc64_personality,ppc64_personality,sys_personality)
SYSCALL(ni_syscall)
SYSCALL_SPU(setfsuid)
SYSCALL_SPU(setfsgid)
SYSCALL_SPU(llseek)			/* 140 */
COMPAT_SYS_SPU(getdents)
SYSX_SPU(sys_select,ppc32_select,sys_select)
SYSCALL_SPU(flock)
SYSCALL_SPU(msync)
COMPAT_SYS_SPU(readv)		/* 145 */
COMPAT_SYS_SPU(writev)
COMPAT_SYS_SPU(getsid)
SYSCALL_SPU(fdatasync)
COMPAT_SYS(sysctl)
SYSCALL_SPU(mlock)			/* 150 */
SYSCALL_SPU(munlock)
SYSCALL_SPU(mlockall)
SYSCALL_SPU(munlockall)
COMPAT_SYS_SPU(sched_setparam)
COMPAT_SYS_SPU(sched_getparam)	/* 155 */
COMPAT_SYS_SPU(sched_setscheduler)
COMPAT_SYS_SPU(sched_getscheduler)
SYSCALL_SPU(sched_yield)
COMPAT_SYS_SPU(sched_get_priority_max)
COMPAT_SYS_SPU(sched_get_priority_min)	/* 160 */
COMPAT_SYS_SPU(sched_rr_get_interval)
COMPAT_SYS_SPU(nanosleep)
SYSCALL_SPU(mremap)
SYSCALL_SPU(setresuid)
SYSCALL_SPU(getresuid)		/* 165 */
SYSCALL(ni_syscall)
SYSCALL_SPU(poll)
SYSCALL(ni_syscall)
SYSCALL_SPU(setresgid)
SYSCALL_SPU(getresgid)		/* 170 */
COMPAT_SYS_SPU(prctl)
COMPAT_SYS(rt_sigreturn)
COMPAT_SYS(rt_sigaction)
COMPAT_SYS(rt_sigprocmask)
COMPAT_SYS(rt_sigpending)	/* 175 */
COMPAT_SYS(rt_sigtimedwait)
COMPAT_SYS(rt_sigqueueinfo)
COMPAT_SYS(rt_sigsuspend)
COMPAT_SYS_SPU(pread64)
COMPAT_SYS_SPU(pwrite64)	/* 180 */
SYSCALL_SPU(chown)
SYSCALL_SPU(getcwd)
SYSCALL_SPU(capget)
SYSCALL_SPU(capset)
COMPAT_SYS(sigaltstack)		/* 185 */
SYSX_SPU(sys_sendfile64,compat_sys_sendfile,sys_sendfile)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
PPC_SYS(vfork)
COMPAT_SYS_SPU(getrlimit)	/* 190 */
COMPAT_SYS_SPU(readahead)
SYS32ONLY(mmap2)
SYS32ONLY(truncate64)
SYS32ONLY(ftruncate64)
SYSX(sys_ni_syscall,sys_stat64,sys_stat64)	/* 195 */
SYSX(sys_ni_syscall,sys_lstat64,sys_lstat64)
SYSX(sys_ni_syscall,sys_fstat64,sys_fstat64)
SYSCALL(pciconfig_read)
SYSCALL(pciconfig_write)
SYSCALL(pciconfig_iobase)	/* 200 */
SYSCALL(ni_syscall)
SYSCALL_SPU(getdents64)
SYSCALL_SPU(pivot_root)
SYSX(sys_ni_syscall,compat_sys_fcntl64,sys_fcntl64)
SYSCALL_SPU(madvise)		/* 205 */
SYSCALL_SPU(mincore)
SYSCALL_SPU(gettid)
SYSCALL_SPU(tkill)
SYSCALL_SPU(setxattr)
SYSCALL_SPU(lsetxattr)		/* 210 */
SYSCALL_SPU(fsetxattr)
SYSCALL_SPU(getxattr)
SYSCALL_SPU(lgetxattr)
SYSCALL_SPU(fgetxattr)
SYSCALL_SPU(listxattr)		/*  215 */
SYSCALL_SPU(llistxattr)
SYSCALL_SPU(flistxattr)
SYSCALL_SPU(removexattr)
SYSCALL_SPU(lremovexattr)
SYSCALL_SPU(fremovexattr)	/* 220 */
COMPAT_SYS_SPU(futex)
COMPAT_SYS_SPU(sched_setaffinity)
COMPAT_SYS_SPU(sched_getaffinity)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 225 */
SYS32ONLY(sendfile64)
COMPAT_SYS_SPU(io_setup)
SYSCALL_SPU(io_destroy)
COMPAT_SYS_SPU(io_getevents)
COMPAT_SYS_SPU(io_submit)	/* 230 */
SYSCALL_SPU(io_cancel)
SYSCALL(set_tid_address)
SYSX_SPU(sys_fadvise64,ppc32_fadvise64,sys_fadvise64)
SYSCALL(exit_group)
SYSX(sys_lookup_dcookie,ppc32_lookup_dcookie,sys_lookup_dcookie)	/* 235 */
SYSCALL_SPU(epoll_create)
SYSCALL_SPU(epoll_ctl)
SYSCALL_SPU(epoll_wait)
SYSCALL_SPU(remap_file_pages)
SYSX_SPU(sys_timer_create,compat_sys_timer_create,sys_timer_create) /* 240 */
COMPAT_SYS_SPU(timer_settime)
COMPAT_SYS_SPU(timer_gettime)
SYSCALL_SPU(timer_getoverrun)
SYSCALL_SPU(timer_delete)
COMPAT_SYS_SPU(clock_settime)	/* 245 */
COMPAT_SYS_SPU(clock_gettime)
COMPAT_SYS_SPU(clock_getres)
COMPAT_SYS_SPU(clock_nanosleep)
SYSX(ppc64_swapcontext,ppc32_swapcontext,ppc_swapcontext)
COMPAT_SYS_SPU(tgkill)		/* 250 */
COMPAT_SYS_SPU(utimes)
COMPAT_SYS_SPU(statfs64)
COMPAT_SYS_SPU(fstatfs64)
SYSX(sys_ni_syscall, ppc_fadvise64_64, ppc_fadvise64_64)
PPC_SYS_SPU(rtas)			/* 255 */
OLDSYS(debug_setcontext)
SYSCALL(ni_syscall)
COMPAT_SYS(migrate_pages)
COMPAT_SYS(mbind)
COMPAT_SYS(get_mempolicy)	/* 260 */
COMPAT_SYS(set_mempolicy)
COMPAT_SYS(mq_open)
SYSCALL(mq_unlink)
COMPAT_SYS(mq_timedsend)
COMPAT_SYS(mq_timedreceive)	/* 265 */
COMPAT_SYS(mq_notify)
COMPAT_SYS(mq_getsetattr)
COMPAT_SYS(kexec_load)
COMPAT_SYS(add_key)
COMPAT_SYS(request_key)		/* 270 */
COMPAT_SYS(keyctl)
COMPAT_SYS(waitid)
COMPAT_SYS(ioprio_set)
COMPAT_SYS(ioprio_get)
SYSCALL(inotify_init)		/* 275 */
SYSCALL(inotify_add_watch)
SYSCALL(inotify_rm_watch)
SYSCALL(spu_run)
SYSCALL(spu_create)
COMPAT_SYS(pselect6)		/* 280 */
COMPAT_SYS(ppoll)
SYSCALL_SPU(unshare)
SYSCALL_SPU(splice)
SYSCALL_SPU(tee)
COMPAT_SYS_SPU(vmsplice)	/* 285 */
COMPAT_SYS_SPU(openat)
SYSCALL_SPU(mkdirat)
SYSCALL_SPU(mknodat)
SYSCALL_SPU(fchownat)
COMPAT_SYS_SPU(futimesat)	/* 290 */
SYSX_SPU(sys_newfstatat, sys_fstatat64, sys_fstatat64)
SYSCALL_SPU(unlinkat)
SYSCALL_SPU(renameat)
SYSCALL_SPU(linkat)
SYSCALL_SPU(symlinkat)		/* 295 */
SYSCALL_SPU(readlinkat)
SYSCALL_SPU(fchmodat)
SYSCALL_SPU(faccessat)
COMPAT_SYS_SPU(get_robust_list)
COMPAT_SYS_SPU(set_robust_list) /* 300 */
COMPAT_SYS_SPU(move_pages)
SYSCALL_SPU(getcpu)
COMPAT_SYS(epoll_pwait)
COMPAT_SYS_SPU(utimensat)
COMPAT_SYS_SPU(signalfd)	/* 305 */
SYSCALL_SPU(timerfd_create)
SYSCALL_SPU(eventfd)
COMPAT_SYS_SPU(sync_file_range2)
COMPAT_SYS(fallocate)
SYSCALL(subpage_prot)		/* 310 */
COMPAT_SYS_SPU(timerfd_settime)
COMPAT_SYS_SPU(timerfd_gettime)
COMPAT_SYS_SPU(signalfd4)
SYSCALL_SPU(eventfd2)
SYSCALL_SPU(epoll_create1)	/* 315 */
SYSCALL_SPU(dup3)
SYSCALL_SPU(pipe2)
SYSCALL(inotify_init1)
SYSCALL_SPU(perf_event_open)
COMPAT_SYS_SPU(preadv)		/* 320 */
COMPAT_SYS_SPU(pwritev)
COMPAT_SYS(rt_tgsigqueueinfo)
SYSCALL(fanotify_init)
COMPAT_SYS(fanotify_mark)
SYSCALL_SPU(prlimit64)	/* 325 */
SYSCALL_SPU(socket)
SYSCALL_SPU(bind)
SYSCALL_SPU(connect)
SYSCALL_SPU(listen)
SYSCALL_SPU(accept)		/* 330 */
SYSCALL_SPU(getsockname)
SYSCALL_SPU(getpeername)
SYSCALL_SPU(socketpair)
SYSCALL_SPU(send)
SYSCALL_SPU(sendto)		/* 335 */
COMPAT_SYS_SPU(recv)
COMPAT_SYS_SPU(recvfrom)
SYSCALL_SPU(shutdown)
COMPAT_SYS_SPU(setsockopt)
COMPAT_SYS_SPU(getsockopt)  /* 340 */
COMPAT_SYS_SPU(sendmsg)
COMPAT_SYS_SPU(recvmsg)
COMPAT_SYS_SPU(recvmmsg)
SYSCALL_SPU(accept4)
SYSCALL_SPU(name_to_handle_at)
COMPAT_SYS_SPU(open_by_handle_at)
COMPAT_SYS_SPU(clock_adjtime)
SYSCALL_SPU(syncfs)
SYSCALL(sendmmsg)
SYSCALL(setns)				/* 350 */
COMPAT_SYS(process_vm_readv)
COMPAT_SYS(process_vm_writev)
#ifdef MY_ABC_HERE
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 355 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 360 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 365 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 370 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 375 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 380 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 385 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 390 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 395 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
#ifdef MY_ABC_HERE
SYSCALL(SYNOUtime)             /* 402 */
#else
SYSCALL(ni_syscall)
#endif
#ifdef MY_ABC_HERE
SYSCALL(SYNOArchiveBit)        /* 403 */
#else
SYSCALL(ni_syscall)
#endif
#ifdef MY_ABC_HERE
SYSCALL(recvfile)              /* 404 */
#else
SYSCALL(ni_syscall)
#endif
#ifdef MY_ABC_HERE
SYSCALL(SYNOMTDAlloc)		/* 405 */
#else
SYSCALL(ni_syscall)
#endif
#ifdef MY_ABC_HERE
SYSCALL(SYNOCaselessStat64)            /* 406 */
SYSCALL(SYNOCaselessLStat64)           /* 407 */
SYSCALL(SYNOCaselessStat)              /* 408 */
SYSCALL(SYNOCaselessLStat)             /* 409 */
#else
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
#endif
#ifdef MY_ABC_HERE
SYSCALL(SYNOEcryptName)		/* 410 */
SYSCALL(SYNODecryptName)	/* 411 */
#else
SYSCALL(ni_syscall)			/* 410 */
SYSCALL(ni_syscall)
#endif
#ifdef MY_ABC_HERE
SYSCALL(SYNOACLCheckPerm)		/* 412 */
SYSCALL(SYNOACLIsSupport)		/* 413 */
SYSCALL(SYNOACLGetPerm)		    /* 414 */
#else
SYSCALL(ni_syscall)			/* 412 */	
SYSCALL(ni_syscall)			
SYSCALL(ni_syscall)
#endif
SYSCALL(ni_syscall)
#ifdef MY_ABC_HERE
SYSCALL(SYNOStat)  			 /* 416 */
SYSCALL(SYNOFStat)  		 /* 417 */
SYSCALL(SYNOLStat)  		 /* 418 */
SYSCALL(SYNOStat64)             /* 419 */
SYSCALL(SYNOFStat64)            /* 420 */
SYSCALL(SYNOLStat64)            /* 421 */
#else
SYSCALL(ni_syscall)          /* 416 */
SYSCALL(ni_syscall)          /* 417 */
SYSCALL(ni_syscall)          /* 418 */
SYSCALL(ni_syscall)			 /* 419 */
SYSCALL(ni_syscall)			 /* 420 */
SYSCALL(ni_syscall)			 /* 421 */
#endif /* MY_ABC_HERE */
#ifdef CONFIG_SYNO_NOTIFY
SYSCALL(SYNONotifyInit) /* 422 */
SYSCALL(SYNONotifyAddWatch) /* 423 */
SYSCALL(SYNONotifyRemoveWatch) /* 424 */
SYSCALL(SYNONotifyAddWatch32) /* 425 */
SYSCALL(SYNONotifyRemoveWatch32) /* 426 */
#else
SYSCALL(ni_syscall) /* 422 */
SYSCALL(ni_syscall) /* 423 */
SYSCALL(ni_syscall) /* 424 */
SYSCALL(ni_syscall) /* 425 */
SYSCALL(ni_syscall) /* 426 */
#endif /* CONFIG_SYNO_NOTIFY */
#ifdef MY_ABC_HERE
SYSCALL(SYNOArchiveOverwrite) /* 427 */
#else
SYSCALL(ni_syscall)			/* 427 */
#endif
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 430 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 435 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 440 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)			/* 445 */
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
SYSCALL(ni_syscall)
#endif
