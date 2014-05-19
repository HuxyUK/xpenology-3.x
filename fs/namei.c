/*
 *  linux/fs/namei.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */

/*
 * Some corrections by tytso.
 */

/* [Feb 1997 T. Schoebel-Theuer] Complete rewrite of the pathname
 * lookup logic.
 */
/* [Feb-Apr 2000, AV] Rewrite to the new namespace architecture.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/pagemap.h>
#include <linux/fsnotify.h>
#include <linux/personality.h>
#include <linux/security.h>
#include <linux/ima.h>
#include <linux/syscalls.h>
#include <linux/mount.h>
#include <linux/audit.h>
#include <linux/capability.h>
#include <linux/file.h>
#include <linux/fcntl.h>
#include <linux/device_cgroup.h>
#include <linux/fs_struct.h>
#include <linux/posix_acl.h>
#include <asm/uaccess.h>

#ifdef CONFIG_FS_SYNO_ACL
#include "synoacl_int.h"
#endif
#include "internal.h"

#ifdef MY_ABC_HERE
int SYNOUnicodeUTF8ChrToUTF16Chr(u_int16_t *p, const u_int8_t *s, int n);
int SYNOUnicodeUTF16ChrToUTF8Chr(u_int8_t *s, u_int16_t wc, int maxlen);
u_int16_t *SYNOUnicodeGenerateDefaultUpcaseTable(void);
u_int16_t *DefUpcaseTable(void);


/*
 * Sample implementation from Unicode home page.
 * http://www.stonehand.com/unicode/standard/fss-utf.html
 */
struct utf8_table {
	int     cmask;
	int     cval;
	int     shift;
	long    lmask;
	long    lval;
};

static struct utf8_table utf8_table[] =
{
    {0x80,  0x00,   0*6,    0x7F,           0,         /* 1 byte sequence */},
    {0xE0,  0xC0,   1*6,    0x7FF,          0x80,      /* 2 byte sequence */},
    {0xF0,  0xE0,   2*6,    0xFFFF,         0x800,     /* 3 byte sequence */},
    {0xF8,  0xF0,   3*6,    0x1FFFFF,       0x10000,   /* 4 byte sequence */},
    {0xFC,  0xF8,   4*6,    0x3FFFFFF,      0x200000,  /* 5 byte sequence */},
    {0xFE,  0xFC,   5*6,    0x7FFFFFFF,     0x4000000, /* 6 byte sequence */},
    {0,						       /* end of table    */}
};

int SYNOUnicodeUTF8ChrToUTF16Chr(u_int16_t *p, const u_int8_t *s, int n)
{
	long l;
	int c0, c, nc;
	struct utf8_table *t;

	nc = 0;
	c0 = *s;
	l = c0;
	for (t = utf8_table; t->cmask; t++) {
		nc++;
		if ((c0 & t->cmask) == t->cval) {
			l &= t->lmask;
			if (l < t->lval)
				return -1;
			*p = l;
			return nc;
		}
		if (n <= nc)
			return -1;
		s++;
		c = (*s ^ 0x80) & 0xFF;
		if (c & 0xC0)
			return -1;
		l = (l << 6) | c;
	}
	return -1;
}

int SYNOUnicodeUTF16ChrToUTF8Chr(u_int8_t *s, u_int16_t wc, int maxlen)
{
	long l;
	int c, nc;
	struct utf8_table *t;

	if (s == 0)
		return 0;

	l = wc;
	nc = 0;
	for (t = utf8_table; t->cmask && maxlen; t++, maxlen--) {
		nc++;
		if (l <= t->lmask) {
			c = t->shift;
			*s = t->cval | (l >> c);
			while (c > 0) {
				c -= 6;
				s++;
				*s = 0x80 | ((l >> c) & 0x3F);
			}
			return nc;
		}
	}
	return -1;
}


/*
 * upcase.c - Generate the full NTFS Unicode upcase table in little endian.
 *	      Part of the Linux-NTFS project.
 *
 * Copyright (C) 2001 Richard Russon <ntfs@flatcap.org>
 * Copyright (c) 2001,2002 Anton Altaparmakov
 *
 * Modified for mkntfs inclusion 9 June 2001 by Anton Altaparmakov.
 * Modified for kernel inclusion 10 September 2001 by Anton Altparmakov.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program (in the main directory of the Linux-NTFS source
 * in the file COPYING); if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

static u_int16_t gUC[UTF16_UPCASE_TABLE_SIZE];

u_int16_t *SYNOUnicodeGenerateDefaultUpcaseTable(void)
{
	const int uc_run_table[][3] = { /* Start, End, Add */
	{0x0061, 0x007B,  -32}, {0x0451, 0x045D, -80}, {0x1F70, 0x1F72,  74},
	{0x00E0, 0x00F7,  -32}, {0x045E, 0x0460, -80}, {0x1F72, 0x1F76,  86},
	{0x00F8, 0x00FF,  -32}, {0x0561, 0x0587, -48}, {0x1F76, 0x1F78, 100},
	{0x0256, 0x0258, -205}, {0x1F00, 0x1F08,   8}, {0x1F78, 0x1F7A, 128},
	{0x028A, 0x028C, -217}, {0x1F10, 0x1F16,   8}, {0x1F7A, 0x1F7C, 112},
	{0x03AC, 0x03AD,  -38}, {0x1F20, 0x1F28,   8}, {0x1F7C, 0x1F7E, 126},
	{0x03AD, 0x03B0,  -37}, {0x1F30, 0x1F38,   8}, {0x1FB0, 0x1FB2,   8},
	{0x03B1, 0x03C2,  -32}, {0x1F40, 0x1F46,   8}, {0x1FD0, 0x1FD2,   8},
	{0x03C2, 0x03C3,  -31}, {0x1F51, 0x1F52,   8}, {0x1FE0, 0x1FE2,   8},
	{0x03C3, 0x03CC,  -32}, {0x1F53, 0x1F54,   8}, {0x1FE5, 0x1FE6,   7},
	{0x03CC, 0x03CD,  -64}, {0x1F55, 0x1F56,   8}, {0x2170, 0x2180, -16},
	{0x03CD, 0x03CF,  -63}, {0x1F57, 0x1F58,   8}, {0x24D0, 0x24EA, -26},
	{0x0430, 0x0450,  -32}, {0x1F60, 0x1F68,   8}, {0xFF41, 0xFF5B, -32},
	{0}
	};

	const int uc_dup_table[][2] = { /* Start, End */
	{0x0100, 0x012F}, {0x01A0, 0x01A6}, {0x03E2, 0x03EF}, {0x04CB, 0x04CC},
	{0x0132, 0x0137}, {0x01B3, 0x01B7}, {0x0460, 0x0481}, {0x04D0, 0x04EB},
	{0x0139, 0x0149}, {0x01CD, 0x01DD}, {0x0490, 0x04BF}, {0x04EE, 0x04F5},
	{0x014A, 0x0178}, {0x01DE, 0x01EF}, {0x04BF, 0x04BF}, {0x04F8, 0x04F9},
	{0x0179, 0x017E}, {0x01F4, 0x01F5}, {0x04C1, 0x04C4}, {0x1E00, 0x1E95},
	{0x018B, 0x018B}, {0x01FA, 0x0218}, {0x04C7, 0x04C8}, {0x1EA0, 0x1EF9},
	{0}
	};

	const int uc_word_table[][2] = { /* Offset, Value */
	{0x00FF, 0x0178}, {0x01AD, 0x01AC}, {0x01F3, 0x01F1}, {0x0269, 0x0196},
	{0x0183, 0x0182}, {0x01B0, 0x01AF}, {0x0253, 0x0181}, {0x026F, 0x019C},
	{0x0185, 0x0184}, {0x01B9, 0x01B8}, {0x0254, 0x0186}, {0x0272, 0x019D},
	{0x0188, 0x0187}, {0x01BD, 0x01BC}, {0x0259, 0x018F}, {0x0275, 0x019F},
	{0x018C, 0x018B}, {0x01C6, 0x01C4}, {0x025B, 0x0190}, {0x0283, 0x01A9},
	{0x0192, 0x0191}, {0x01C9, 0x01C7}, {0x0260, 0x0193}, {0x0288, 0x01AE},
	{0x0199, 0x0198}, {0x01CC, 0x01CA}, {0x0263, 0x0194}, {0x0292, 0x01B7},
	{0x01A8, 0x01A7}, {0x01DD, 0x018E}, {0x0268, 0x0197},
	{0}
	};

#ifdef MY_ABC_HERE
	/** This is the difference part from Unicode Uppercase Table generated by ICU
	 *  Since samba/netatalk will call SetToCaseless() and override the default
	 *  Uppercase Table in kernel. It's batter eliminate those conflict as early
	 *  as possible.
     */
	const int uc_icu_table[][2] = {	/* Offset, Value */
	{0x00B5, 0x039C}, {0x0131, 0x0049}, {0x017F, 0x0053}, {0x0195, 0x01F6},
	{0x019E, 0x0220}, {0x01BF, 0x01F7}, {0x01C5, 0x01C4}, {0x01C8, 0x01C7},
	{0x01CB, 0x01CA}, {0x01F2, 0x01F1}, {0x01F9, 0x01F8}, {0x0219, 0x0218},
	{0x021B, 0x021A}, {0x021D, 0x021C}, {0x021F, 0x021E}, {0x0223, 0x0222},
	{0x0225, 0x0224}, {0x0227, 0x0226}, {0x0229, 0x0228}, {0x022B, 0x022A},
	{0x022D, 0x022C}, {0x022F, 0x022E}, {0x0231, 0x0230}, {0x0233, 0x0232},
	{0x0280, 0x01A6}, {0x0345, 0x0399}, {0x03D0, 0x0392}, {0x03D1, 0x0398},
	{0x03D5, 0x03A6}, {0x03D6, 0x03A0}, {0x03D9, 0x03D8}, {0x03DB, 0x03DA},
	{0x03DD, 0x03DC}, {0x03DF, 0x03DE}, {0x03E1, 0x03E0}, {0x03F0, 0x039A},
	{0x03F1, 0x03A1}, {0x03F2, 0x03A3}, {0x03F5, 0x0395}, {0x0450, 0x0400},
	{0x045D, 0x040D}, {0x048B, 0x048A}, {0x048D, 0x048C}, {0x048F, 0x048E},
	{0x04C6, 0x04C5}, {0x04CA, 0x04C9}, {0x04CE, 0x04CD}, {0x04ED, 0x04EC},
	{0x0501, 0x0500}, {0x0503, 0x0502}, {0x0505, 0x0504}, {0x0507, 0x0506},
	{0x0509, 0x0508}, {0x050B, 0x050A}, {0x050D, 0x050C}, {0x050F, 0x050E},
	{0x1E9B, 0x1E60}, {0x1FBE, 0x0399},
	{0}
	};
#endif /* MY_ABC_HERE */
	int i, r;
	u_int16_t *uc;

	uc = gUC;

	memset(uc, 0, UTF16_UPCASE_TABLE_SIZE * sizeof(u_int16_t));

	for (i = 0; i < UTF16_UPCASE_TABLE_SIZE; i++)
		uc[i] = i;
	for (r = 0; uc_run_table[r][0]; r++)
		for (i = uc_run_table[r][0]; i < uc_run_table[r][1]; i++)
			uc[i] = uc[i] + uc_run_table[r][2];
	for (r = 0; uc_dup_table[r][0]; r++)
		for (i = uc_dup_table[r][0]; i < uc_dup_table[r][1]; i += 2)
			uc[i + 1] = uc[i + 1] - 1;
	for (r = 0; uc_word_table[r][0]; r++)
		uc[uc_word_table[r][0]] = uc_word_table[r][1];
#ifdef MY_ABC_HERE
	for (r = 0; uc_icu_table[r][0]; r++)
		uc[uc_icu_table[r][0]] = uc_icu_table[r][1];
#endif /* MY_ABC_HERE */
	return uc;
}


static u_int16_t *UpcaseTable = NULL;

u_int16_t *DefUpcaseTable()
{
    if(UpcaseTable==NULL)
        UpcaseTable = SYNOUnicodeGenerateDefaultUpcaseTable();

    return UpcaseTable;
}

int SYNOUnicodeUTF8toUpper(u_int8_t *to,const u_int8_t *from, int maxlen, int clenfrom, u_int16_t *upcasetable)
{
	u_int16_t *UpcaseTbl;
	u_int16_t wc;
	u_int8_t *op;
	int size;

	UpcaseTbl = (upcasetable==NULL) ? DefUpcaseTable() : upcasetable;

	op = to;
	while (clenfrom && maxlen) {
		size = SYNOUnicodeUTF8ChrToUTF16Chr(&wc, from, clenfrom);
		if (size == -1) {
			from++;
			clenfrom--;
			continue;
		} else {
			from += size;
			clenfrom -= size;
		}
		size = SYNOUnicodeUTF16ChrToUTF8Chr(op, UpcaseTbl[wc], maxlen);
		if (size == -1) {
			continue;
		} else {
			op += size;
			maxlen -= size;
		}
	}
	*op = 0;
	return (op - to);
}
EXPORT_SYMBOL(SYNOUnicodeUTF8toUpper);

int SYNOUnicodeUTF8Strcmp(const u_int8_t *utf8str1,const u_int8_t *utf8str2,int clenUtf8Str1, int clenUtf8Str2, u_int16_t *upcasetable)
{
	u_int16_t *UpcaseTbl;
	u_int16_t wc1, wc2;
	int size1, size2;
	int result = -1;

	UpcaseTbl = (upcasetable==NULL) ? DefUpcaseTable() : upcasetable;

	while (clenUtf8Str1 && clenUtf8Str2) {
		size1 = SYNOUnicodeUTF8ChrToUTF16Chr(&wc1, utf8str1, clenUtf8Str1);
		size2 = SYNOUnicodeUTF8ChrToUTF16Chr(&wc2, utf8str2, clenUtf8Str2);

		if (size1 != -1 && size2 != -1) {
			if (UpcaseTbl[wc1] != UpcaseTbl[wc2])
				goto END;
		} else if (size1 == -1 && size2 == -1) {
			if (*utf8str1 != *utf8str2)
				goto END;
			size1 = size2 = 1;
		} else {
			goto END;
		}
		utf8str1 += size1;
		clenUtf8Str1 -= size1;
		utf8str2 += size2;
		clenUtf8Str2 -= size2;
	}
	if (clenUtf8Str1 == 0 && clenUtf8Str2 == 0)
		result = 0;
END:
	return result;
}
EXPORT_SYMBOL(SYNOUnicodeUTF8Strcmp);

asmlinkage long sys_SYNOUnicodeLoadTbl(u_int16_t *rgUCTable)
{
    /* Original, we allow user space to change the unicode table.
	 * So different application can have different table. But actually,
	 * every application should use the same table and our kernel
	 * only saves 1 table.
	 *
	 * Therefore, it is no need to allow user space program to change
	 * unicode table. So use just return 0 here.
	 */

    return 0;
}

#endif /* MY_ABC_HERE */

/* [Feb-1997 T. Schoebel-Theuer]
 * Fundamental changes in the pathname lookup mechanisms (namei)
 * were necessary because of omirr.  The reason is that omirr needs
 * to know the _real_ pathname, not the user-supplied one, in case
 * of symlinks (and also when transname replacements occur).
 *
 * The new code replaces the old recursive symlink resolution with
 * an iterative one (in case of non-nested symlink chains).  It does
 * this with calls to <fs>_follow_link().
 * As a side effect, dir_namei(), _namei() and follow_link() are now 
 * replaced with a single function lookup_dentry() that can handle all 
 * the special cases of the former code.
 *
 * With the new dcache, the pathname is stored at each inode, at least as
 * long as the refcount of the inode is positive.  As a side effect, the
 * size of the dcache depends on the inode cache and thus is dynamic.
 *
 * [29-Apr-1998 C. Scott Ananian] Updated above description of symlink
 * resolution to correspond with current state of the code.
 *
 * Note that the symlink resolution is not *completely* iterative.
 * There is still a significant amount of tail- and mid- recursion in
 * the algorithm.  Also, note that <fs>_readlink() is not used in
 * lookup_dentry(): lookup_dentry() on the result of <fs>_readlink()
 * may return different results than <fs>_follow_link().  Many virtual
 * filesystems (including /proc) exhibit this behavior.
 */

/* [24-Feb-97 T. Schoebel-Theuer] Side effects caused by new implementation:
 * New symlink semantics: when open() is called with flags O_CREAT | O_EXCL
 * and the name already exists in form of a symlink, try to create the new
 * name indicated by the symlink. The old code always complained that the
 * name already exists, due to not following the symlink even if its target
 * is nonexistent.  The new semantics affects also mknod() and link() when
 * the name is a symlink pointing to a non-existent name.
 *
 * I don't know which semantics is the right one, since I have no access
 * to standards. But I found by trial that HP-UX 9.0 has the full "new"
 * semantics implemented, while SunOS 4.1.1 and Solaris (SunOS 5.4) have the
 * "old" one. Personally, I think the new semantics is much more logical.
 * Note that "ln old new" where "new" is a symlink pointing to a non-existing
 * file does succeed in both HP-UX and SunOs, but not in Solaris
 * and in the old Linux semantics.
 */

/* [16-Dec-97 Kevin Buhr] For security reasons, we change some symlink
 * semantics.  See the comments in "open_namei" and "do_link" below.
 *
 * [10-Sep-98 Alan Modra] Another symlink change.
 */

/* [Feb-Apr 2000 AV] Complete rewrite. Rules for symlinks:
 *	inside the path - always follow.
 *	in the last component in creation/removal/renaming - never follow.
 *	if LOOKUP_FOLLOW passed - follow.
 *	if the pathname has trailing slashes - follow.
 *	otherwise - don't follow.
 * (applied in that order).
 *
 * [Jun 2000 AV] Inconsistent behaviour of open() in case if flags==O_CREAT
 * restored for 2.4. This is the last surviving part of old 4.2BSD bug.
 * During the 2.4 we need to fix the userland stuff depending on it -
 * hopefully we will be able to get rid of that wart in 2.5. So far only
 * XEmacs seems to be relying on it...
 */
/*
 * [Sep 2001 AV] Single-semaphore locking scheme (kudos to David Holland)
 * implemented.  Let's see if raised priority of ->s_vfs_rename_mutex gives
 * any extra contention...
 */

/* In order to reduce some races, while at the same time doing additional
 * checking and hopefully speeding things up, we copy filenames to the
 * kernel data space before using them..
 *
 * POSIX.1 2.4: an empty pathname is invalid (ENOENT).
 * PATH_MAX includes the nul terminator --RR.
 */
static int do_getname(const char __user *filename, char *page)
{
	int retval;
	unsigned long len = PATH_MAX;

	if (!segment_eq(get_fs(), KERNEL_DS)) {
		if ((unsigned long) filename >= TASK_SIZE)
			return -EFAULT;
		if (TASK_SIZE - (unsigned long) filename < PATH_MAX)
			len = TASK_SIZE - (unsigned long) filename;
	}

	retval = strncpy_from_user(page, filename, len);
	if (retval > 0) {
		if (retval < len)
			return 0;
		return -ENAMETOOLONG;
	} else if (!retval)
		retval = -ENOENT;
	return retval;
}

static char *getname_flags(const char __user *filename, int flags, int *empty)
{
	char *tmp, *result;

	result = ERR_PTR(-ENOMEM);
	tmp = __getname();
	if (tmp)  {
		int retval = do_getname(filename, tmp);

		result = tmp;
		if (retval < 0) {
			if (retval == -ENOENT && empty)
				*empty = 1;
			if (retval != -ENOENT || !(flags & LOOKUP_EMPTY)) {
				__putname(tmp);
				result = ERR_PTR(retval);
			}
		}
	}
	audit_getname(result);
	return result;
}

char *getname(const char __user * filename)
{
	return getname_flags(filename, 0, 0);
}

#ifdef CONFIG_AUDITSYSCALL
void putname(const char *name)
{
	if (unlikely(!audit_dummy_context()))
		audit_putname(name);
	else
		__putname(name);
}
EXPORT_SYMBOL(putname);
#endif

static int check_acl(struct inode *inode, int mask)
{
#ifdef CONFIG_FS_POSIX_ACL
	struct posix_acl *acl;

	if (mask & MAY_NOT_BLOCK) {
		acl = get_cached_acl_rcu(inode, ACL_TYPE_ACCESS);
	        if (!acl)
	                return -EAGAIN;
		/* no ->get_acl() calls in RCU mode... */
		if (acl == ACL_NOT_CACHED)
			return -ECHILD;
	        return posix_acl_permission(inode, acl, mask & ~MAY_NOT_BLOCK);
	}

	acl = get_cached_acl(inode, ACL_TYPE_ACCESS);

	/*
	 * A filesystem can force a ACL callback by just never filling the
	 * ACL cache. But normally you'd fill the cache either at inode
	 * instantiation time, or on the first ->get_acl call.
	 *
	 * If the filesystem doesn't have a get_acl() function at all, we'll
	 * just create the negative cache entry.
	 */
	if (acl == ACL_NOT_CACHED) {
	        if (inode->i_op->get_acl) {
			acl = inode->i_op->get_acl(inode, ACL_TYPE_ACCESS);
			if (IS_ERR(acl))
				return PTR_ERR(acl);
		} else {
		        set_cached_acl(inode, ACL_TYPE_ACCESS, NULL);
		        return -EAGAIN;
		}
	}

	if (acl) {
	        int error = posix_acl_permission(inode, acl, mask);
	        posix_acl_release(acl);
	        return error;
	}
#endif

	return -EAGAIN;
}

/*
 * This does the basic permission checking
 */
static int acl_permission_check(struct inode *inode, int mask)
{
	unsigned int mode = inode->i_mode;

	if (current_user_ns() != inode_userns(inode))
		goto other_perms;

	if (likely(current_fsuid() == inode->i_uid))
		mode >>= 6;
	else {
		if (IS_POSIXACL(inode) && (mode & S_IRWXG)) {
			int error = check_acl(inode, mask);
			if (error != -EAGAIN)
				return error;
		}

		if (in_group_p(inode->i_gid))
			mode >>= 3;
	}

other_perms:
	/*
	 * If the DACs are ok we don't need any capability check.
	 */
	if ((mask & ~mode & (MAY_READ | MAY_WRITE | MAY_EXEC)) == 0)
		return 0;
	return -EACCES;
}

/**
 * generic_permission -  check for access rights on a Posix-like filesystem
 * @inode:	inode to check access rights for
 * @mask:	right to check for (%MAY_READ, %MAY_WRITE, %MAY_EXEC, ...)
 *
 * Used to check for read/write/execute permissions on a file.
 * We use "fsuid" for this, letting us set arbitrary permissions
 * for filesystem access without changing the "normal" uids which
 * are used for other things.
 *
 * generic_permission is rcu-walk aware. It returns -ECHILD in case an rcu-walk
 * request cannot be satisfied (eg. requires blocking or too much complexity).
 * It would then be called again in ref-walk mode.
 */
int generic_permission(struct inode *inode, int mask)
{
	int ret;

	/*
	 * Do the basic permission checks.
	 */
	ret = acl_permission_check(inode, mask);
	if (ret != -EACCES)
		return ret;

	if (S_ISDIR(inode->i_mode)) {
		/* DACs are overridable for directories */
		if (ns_capable(inode_userns(inode), CAP_DAC_OVERRIDE))
			return 0;
		if (!(mask & MAY_WRITE))
			if (ns_capable(inode_userns(inode), CAP_DAC_READ_SEARCH))
				return 0;
		return -EACCES;
	}
	/*
	 * Read/write DACs are always overridable.
	 * Executable DACs are overridable when there is
	 * at least one exec bit set.
	 */
	if (!(mask & MAY_EXEC) || (inode->i_mode & S_IXUGO))
		if (ns_capable(inode_userns(inode), CAP_DAC_OVERRIDE))
			return 0;

	/*
	 * Searching includes executable on directories, else just read.
	 */
	mask &= MAY_READ | MAY_WRITE | MAY_EXEC;
	if (mask == MAY_READ)
		if (ns_capable(inode_userns(inode), CAP_DAC_READ_SEARCH))
			return 0;

	return -EACCES;
}

/*
 * We _really_ want to just do "generic_permission()" without
 * even looking at the inode->i_op values. So we keep a cache
 * flag in inode->i_opflags, that says "this has not special
 * permission function, use the fast case".
 */
static inline int do_inode_permission(struct inode *inode, int mask)
{
	if (unlikely(!(inode->i_opflags & IOP_FASTPERM))) {
		if (likely(inode->i_op->permission))
			return inode->i_op->permission(inode, mask);

		/* This gets set once for the inode lifetime */
		spin_lock(&inode->i_lock);
		inode->i_opflags |= IOP_FASTPERM;
		spin_unlock(&inode->i_lock);
	}
	return generic_permission(inode, mask);
}

/**
 * inode_permission  -  check for access rights to a given inode
 * @inode:	inode to check permission on
 * @mask:	right to check for (%MAY_READ, %MAY_WRITE, %MAY_EXEC, ...)
 *
 * Used to check for read/write/execute permissions on an inode.
 * We use "fsuid" for this, letting us set arbitrary permissions
 * for filesystem access without changing the "normal" uids which
 * are used for other things.
 *
 * When checking for MAY_APPEND, MAY_WRITE must also be set in @mask.
 */
int inode_permission(struct inode *inode, int mask)
{
	int retval;

	if (unlikely(mask & MAY_WRITE)) {
		umode_t mode = inode->i_mode;

		/*
		 * Nobody gets write access to a read-only fs.
		 */
		if (IS_RDONLY(inode) &&
		    (S_ISREG(mode) || S_ISDIR(mode) || S_ISLNK(mode)))
			return -EROFS;

		/*
		 * Nobody gets write access to an immutable file.
		 */
		if (IS_IMMUTABLE(inode))
			return -EACCES;
	}

	retval = do_inode_permission(inode, mask);
	if (retval)
		return retval;

	retval = devcgroup_inode_permission(inode, mask);
	if (retval)
		return retval;

	return security_inode_permission(inode, mask);
}

/**
 * path_get - get a reference to a path
 * @path: path to get the reference to
 *
 * Given a path increment the reference count to the dentry and the vfsmount.
 */
void path_get(struct path *path)
{
	mntget(path->mnt);
	dget(path->dentry);
}
EXPORT_SYMBOL(path_get);

/**
 * path_put - put a reference to a path
 * @path: path to put the reference to
 *
 * Given a path decrement the reference count to the dentry and the vfsmount.
 */
void path_put(struct path *path)
{
	dput(path->dentry);
	mntput(path->mnt);
}
EXPORT_SYMBOL(path_put);

/*
 * Path walking has 2 modes, rcu-walk and ref-walk (see
 * Documentation/filesystems/path-lookup.txt).  In situations when we can't
 * continue in RCU mode, we attempt to drop out of rcu-walk mode and grab
 * normal reference counts on dentries and vfsmounts to transition to rcu-walk
 * mode.  Refcounts are grabbed at the last known good point before rcu-walk
 * got stuck, so ref-walk may continue from there. If this is not successful
 * (eg. a seqcount has changed), then failure is returned and it's up to caller
 * to restart the path walk from the beginning in ref-walk mode.
 */

/**
 * unlazy_walk - try to switch to ref-walk mode.
 * @nd: nameidata pathwalk data
 * @dentry: child of nd->path.dentry or NULL
 * Returns: 0 on success, -ECHILD on failure
 *
 * unlazy_walk attempts to legitimize the current nd->path, nd->root and dentry
 * for ref-walk mode.  @dentry must be a path found by a do_lookup call on
 * @nd or NULL.  Must be called from rcu-walk context.
 */
static int unlazy_walk(struct nameidata *nd, struct dentry *dentry)
{
	struct fs_struct *fs = current->fs;
	struct dentry *parent = nd->path.dentry;
	int want_root = 0;

	BUG_ON(!(nd->flags & LOOKUP_RCU));
	if (nd->root.mnt && !(nd->flags & LOOKUP_ROOT)) {
		want_root = 1;
		spin_lock(&fs->lock);
		if (nd->root.mnt != fs->root.mnt ||
				nd->root.dentry != fs->root.dentry)
			goto err_root;
	}
	spin_lock(&parent->d_lock);
	if (!dentry) {
		if (!__d_rcu_to_refcount(parent, nd->seq))
			goto err_parent;
		BUG_ON(nd->inode != parent->d_inode);
	} else {
		if (dentry->d_parent != parent)
			goto err_parent;
		spin_lock_nested(&dentry->d_lock, DENTRY_D_LOCK_NESTED);
		if (!__d_rcu_to_refcount(dentry, nd->seq))
			goto err_child;
		/*
		 * If the sequence check on the child dentry passed, then
		 * the child has not been removed from its parent. This
		 * means the parent dentry must be valid and able to take
		 * a reference at this point.
		 */
		BUG_ON(!IS_ROOT(dentry) && dentry->d_parent != parent);
		BUG_ON(!parent->d_count);
		parent->d_count++;
		spin_unlock(&dentry->d_lock);
	}
	spin_unlock(&parent->d_lock);
	if (want_root) {
		path_get(&nd->root);
		spin_unlock(&fs->lock);
	}
	mntget(nd->path.mnt);

	rcu_read_unlock();
	br_read_unlock(vfsmount_lock);
	nd->flags &= ~LOOKUP_RCU;
	return 0;

err_child:
	spin_unlock(&dentry->d_lock);
err_parent:
	spin_unlock(&parent->d_lock);
err_root:
	if (want_root)
		spin_unlock(&fs->lock);
	return -ECHILD;
}

/**
 * release_open_intent - free up open intent resources
 * @nd: pointer to nameidata
 */
void release_open_intent(struct nameidata *nd)
{
	struct file *file = nd->intent.open.file;

	if (file && !IS_ERR(file)) {
		if (file->f_path.dentry == NULL)
			put_filp(file);
		else
			fput(file);
	}
}

static inline int d_revalidate(struct dentry *dentry, struct nameidata *nd)
{
	return dentry->d_op->d_revalidate(dentry, nd);
}

/**
 * complete_walk - successful completion of path walk
 * @nd:  pointer nameidata
 *
 * If we had been in RCU mode, drop out of it and legitimize nd->path.
 * Revalidate the final result, unless we'd already done that during
 * the path walk or the filesystem doesn't ask for it.  Return 0 on
 * success, -error on failure.  In case of failure caller does not
 * need to drop nd->path.
 */
static int complete_walk(struct nameidata *nd)
{
	struct dentry *dentry = nd->path.dentry;
	int status;

	if (nd->flags & LOOKUP_RCU) {
		nd->flags &= ~LOOKUP_RCU;
		if (!(nd->flags & LOOKUP_ROOT))
			nd->root.mnt = NULL;
		spin_lock(&dentry->d_lock);
		if (unlikely(!__d_rcu_to_refcount(dentry, nd->seq))) {
			spin_unlock(&dentry->d_lock);
			rcu_read_unlock();
			br_read_unlock(vfsmount_lock);
			return -ECHILD;
		}
		BUG_ON(nd->inode != dentry->d_inode);
		spin_unlock(&dentry->d_lock);
		mntget(nd->path.mnt);
		rcu_read_unlock();
		br_read_unlock(vfsmount_lock);
	}

	if (likely(!(nd->flags & LOOKUP_JUMPED)))
		return 0;

	if (likely(!(dentry->d_flags & DCACHE_OP_REVALIDATE)))
		return 0;

	if (likely(!(dentry->d_sb->s_type->fs_flags & FS_REVAL_DOT)))
		return 0;

	/* Note: we do not d_invalidate() */
	status = d_revalidate(dentry, nd);
	if (status > 0)
		return 0;

	if (!status)
		status = -ESTALE;

	path_put(&nd->path);
	return status;
}

static __always_inline void set_root(struct nameidata *nd)
{
	if (!nd->root.mnt)
		get_fs_root(current->fs, &nd->root);
}

static int link_path_walk(const char *, struct nameidata *);

static __always_inline void set_root_rcu(struct nameidata *nd)
{
	if (!nd->root.mnt) {
		struct fs_struct *fs = current->fs;
		unsigned seq;

		do {
			seq = read_seqcount_begin(&fs->seq);
			nd->root = fs->root;
			nd->seq = __read_seqcount_begin(&nd->root.dentry->d_seq);
		} while (read_seqcount_retry(&fs->seq, seq));
	}
}

static __always_inline int __vfs_follow_link(struct nameidata *nd, const char *link)
{
	int ret;

	if (IS_ERR(link))
		goto fail;

	if (*link == '/') {
		set_root(nd);
		path_put(&nd->path);
		nd->path = nd->root;
		path_get(&nd->root);
		nd->flags |= LOOKUP_JUMPED;
	}
	nd->inode = nd->path.dentry->d_inode;

	ret = link_path_walk(link, nd);
	return ret;
fail:
	path_put(&nd->path);
	return PTR_ERR(link);
}

static void path_put_conditional(struct path *path, struct nameidata *nd)
{
	dput(path->dentry);
	if (path->mnt != nd->path.mnt)
		mntput(path->mnt);
}

static inline void path_to_nameidata(const struct path *path,
					struct nameidata *nd)
{
	if (!(nd->flags & LOOKUP_RCU)) {
		dput(nd->path.dentry);
		if (nd->path.mnt != path->mnt)
			mntput(nd->path.mnt);
	}
	nd->path.mnt = path->mnt;
	nd->path.dentry = path->dentry;
}

static inline void put_link(struct nameidata *nd, struct path *link, void *cookie)
{
	struct inode *inode = link->dentry->d_inode;
	if (!IS_ERR(cookie) && inode->i_op->put_link)
		inode->i_op->put_link(link->dentry, nd, cookie);
	path_put(link);
}

static __always_inline int
follow_link(struct path *link, struct nameidata *nd, void **p)
{
	int error;
	struct dentry *dentry = link->dentry;

	BUG_ON(nd->flags & LOOKUP_RCU);

	if (link->mnt == nd->path.mnt)
		mntget(link->mnt);

	if (unlikely(current->total_link_count >= 40)) {
		*p = ERR_PTR(-ELOOP); /* no ->put_link(), please */
		path_put(&nd->path);
		return -ELOOP;
	}
	cond_resched();
	current->total_link_count++;

	touch_atime(link->mnt, dentry);
	nd_set_link(nd, NULL);

	error = security_inode_follow_link(link->dentry, nd);
	if (error) {
		*p = ERR_PTR(error); /* no ->put_link(), please */
		path_put(&nd->path);
		return error;
	}

	nd->last_type = LAST_BIND;
	*p = dentry->d_inode->i_op->follow_link(dentry, nd);
	error = PTR_ERR(*p);
	if (!IS_ERR(*p)) {
		char *s = nd_get_link(nd);
		error = 0;
		if (s)
			error = __vfs_follow_link(nd, s);
		else if (nd->last_type == LAST_BIND) {
			nd->flags |= LOOKUP_JUMPED;
			nd->inode = nd->path.dentry->d_inode;
			if (nd->inode->i_op->follow_link) {
				/* stepped on a _really_ weird one */
				path_put(&nd->path);
				error = -ELOOP;
			}
		}
	}
	return error;
}

static int follow_up_rcu(struct path *path)
{
	struct vfsmount *parent;
	struct dentry *mountpoint;

	parent = path->mnt->mnt_parent;
	if (parent == path->mnt)
		return 0;
	mountpoint = path->mnt->mnt_mountpoint;
	path->dentry = mountpoint;
	path->mnt = parent;
	return 1;
}

int follow_up(struct path *path)
{
	struct vfsmount *parent;
	struct dentry *mountpoint;

	br_read_lock(vfsmount_lock);
	parent = path->mnt->mnt_parent;
	if (parent == path->mnt) {
		br_read_unlock(vfsmount_lock);
		return 0;
	}
	mntget(parent);
	mountpoint = dget(path->mnt->mnt_mountpoint);
	br_read_unlock(vfsmount_lock);
	dput(path->dentry);
	path->dentry = mountpoint;
	mntput(path->mnt);
	path->mnt = parent;
	return 1;
}

#ifdef CONFIG_SYNO_NOTIFY
/*
   Fetch full mount point path,
   It traverse vfsmount from down to up by following mnt_parent
   @in: struct vfsmount: vfsmount structure, size_t buf_len: path buffer length
   @out: mnt_full_path: full path of vfsmount struct
   @return: -1: failed, 0 : success
 */
int syno_fetch_mountpoint_fullpath(struct vfsmount *mnt, size_t buf_len, char *mnt_full_path)
{
	int ret = -1;
	struct vfsmount *tmp_mnt = NULL;
	char *mnt_dentry_path = NULL;
	char *mnt_dentry_path_buf = NULL;
	char *mnt_full_path_buf = NULL;
	mnt_dentry_path_buf = kmalloc(PATH_MAX, GFP_NOFS);
	mnt_full_path_buf = kmalloc(PATH_MAX, GFP_NOFS);

	if(!mnt_dentry_path_buf || !mnt_full_path_buf)
		goto ERR;

	tmp_mnt = mnt;
	br_read_lock(vfsmount_lock);
	do{
		mnt_dentry_path = dentry_path_raw(tmp_mnt->mnt_mountpoint, mnt_dentry_path_buf, PATH_MAX - 1);
		if(IS_ERR(mnt_dentry_path)){
			br_read_unlock(vfsmount_lock);
			goto ERR;
		}
		snprintf(mnt_full_path_buf, PATH_MAX, "%s", mnt_full_path);
		if(mnt_full_path_buf[0] == '/' && (mnt_dentry_path[0] == '/' && mnt_dentry_path[1] == 0))
			snprintf(mnt_full_path, buf_len, "%s", mnt_full_path_buf);
		else
			snprintf(mnt_full_path, buf_len, "%s%s", mnt_dentry_path, mnt_full_path_buf);
		tmp_mnt = tmp_mnt->mnt_parent;
	}while(tmp_mnt != tmp_mnt->mnt_parent);
	br_read_unlock(vfsmount_lock);
	ret = 0;
ERR:
	kfree(mnt_dentry_path_buf);
	kfree(mnt_full_path_buf);
	return ret;
}
#endif

/*
 * Perform an automount
 * - return -EISDIR to tell follow_managed() to stop and return the path we
 *   were called with.
 */
static int follow_automount(struct path *path, unsigned flags,
			    bool *need_mntput)
{
	struct vfsmount *mnt;
	int err;

	if (!path->dentry->d_op || !path->dentry->d_op->d_automount)
		return -EREMOTE;

	/* We don't want to mount if someone's just doing a stat -
	 * unless they're stat'ing a directory and appended a '/' to
	 * the name.
	 *
	 * We do, however, want to mount if someone wants to open or
	 * create a file of any type under the mountpoint, wants to
	 * traverse through the mountpoint or wants to open the
	 * mounted directory.  Also, autofs may mark negative dentries
	 * as being automount points.  These will need the attentions
	 * of the daemon to instantiate them before they can be used.
	 */
	if (!(flags & (LOOKUP_PARENT | LOOKUP_DIRECTORY |
		     LOOKUP_OPEN | LOOKUP_CREATE | LOOKUP_AUTOMOUNT)) &&
	    path->dentry->d_inode)
		return -EISDIR;

	current->total_link_count++;
	if (current->total_link_count >= 40)
		return -ELOOP;

	mnt = path->dentry->d_op->d_automount(path);
	if (IS_ERR(mnt)) {
		/*
		 * The filesystem is allowed to return -EISDIR here to indicate
		 * it doesn't want to automount.  For instance, autofs would do
		 * this so that its userspace daemon can mount on this dentry.
		 *
		 * However, we can only permit this if it's a terminal point in
		 * the path being looked up; if it wasn't then the remainder of
		 * the path is inaccessible and we should say so.
		 */
		if (PTR_ERR(mnt) == -EISDIR && (flags & LOOKUP_PARENT))
			return -EREMOTE;
		return PTR_ERR(mnt);
	}

	if (!mnt) /* mount collision */
		return 0;

	if (!*need_mntput) {
		/* lock_mount() may release path->mnt on error */
		mntget(path->mnt);
		*need_mntput = true;
	}
	err = finish_automount(mnt, path);

	switch (err) {
	case -EBUSY:
		/* Someone else made a mount here whilst we were busy */
		return 0;
	case 0:
		path_put(path);
		path->mnt = mnt;
		path->dentry = dget(mnt->mnt_root);
		return 0;
	default:
		return err;
	}

}

/*
 * Handle a dentry that is managed in some way.
 * - Flagged for transit management (autofs)
 * - Flagged as mountpoint
 * - Flagged as automount point
 *
 * This may only be called in refwalk mode.
 *
 * Serialization is taken care of in namespace.c
 */
#ifdef MY_ABC_HERE
static int follow_managed(struct path *path, struct nameidata *nd)
#else
static int follow_managed(struct path *path, unsigned flags)
#endif
{
	struct vfsmount *mnt = path->mnt; /* held by caller, must be left alone */
	unsigned managed;
	bool need_mntput = false;
	int ret = 0;
#ifdef MY_ABC_HERE
	int flags = nd->flags;
#endif

	/* Given that we're not holding a lock here, we retain the value in a
	 * local variable for each dentry as we look at it so that we don't see
	 * the components of that value change under us */
	while (managed = ACCESS_ONCE(path->dentry->d_flags),
	       managed &= DCACHE_MANAGED_DENTRY,
	       unlikely(managed != 0)) {
		/* Allow the filesystem to manage the transit without i_mutex
		 * being held. */
		if (managed & DCACHE_MANAGE_TRANSIT) {
			BUG_ON(!path->dentry->d_op);
			BUG_ON(!path->dentry->d_op->d_manage);
			ret = path->dentry->d_op->d_manage(path->dentry, false);
			if (ret < 0)
				break;
		}

		/* Transit to a mounted filesystem. */
		if (managed & DCACHE_MOUNTED) {
			struct vfsmount *mounted = lookup_mnt(path);
			if (mounted) {
				dput(path->dentry);
				if (need_mntput)
					mntput(path->mnt);
				path->mnt = mounted;
				path->dentry = dget(mounted->mnt_root);
				need_mntput = true;
#ifdef MY_ABC_HERE
				nd->flags |= LOOKUP_MOUNTED;
#endif
				continue;
			}

			/* Something is mounted on this dentry in another
			 * namespace and/or whatever was mounted there in this
			 * namespace got unmounted before we managed to get the
			 * vfsmount_lock */
		}

		/* Handle an automount point */
		if (managed & DCACHE_NEED_AUTOMOUNT) {
			ret = follow_automount(path, flags, &need_mntput);
			if (ret < 0)
				break;
			continue;
		}

		/* We didn't change the current path point */
		break;
	}

	if (need_mntput && path->mnt == mnt)
		mntput(path->mnt);
	if (ret == -EISDIR)
		ret = 0;
	return ret < 0 ? ret : need_mntput;
}

int follow_down_one(struct path *path)
{
	struct vfsmount *mounted;

	mounted = lookup_mnt(path);
	if (mounted) {
		dput(path->dentry);
		mntput(path->mnt);
		path->mnt = mounted;
		path->dentry = dget(mounted->mnt_root);
		return 1;
	}
	return 0;
}

static inline bool managed_dentry_might_block(struct dentry *dentry)
{
	return (dentry->d_flags & DCACHE_MANAGE_TRANSIT &&
		dentry->d_op->d_manage(dentry, true) < 0);
}

/*
 * Try to skip to top of mountpoint pile in rcuwalk mode.  Fail if
 * we meet a managed dentry that would need blocking.
 */
static bool __follow_mount_rcu(struct nameidata *nd, struct path *path,
			       struct inode **inode)
{
	for (;;) {
		struct vfsmount *mounted;
		/*
		 * Don't forget we might have a non-mountpoint managed dentry
		 * that wants to block transit.
		 */
		if (unlikely(managed_dentry_might_block(path->dentry)))
			return false;

		if (!d_mountpoint(path->dentry))
			break;

		mounted = __lookup_mnt(path->mnt, path->dentry, 1);
		if (!mounted)
			break;
		path->mnt = mounted;
		path->dentry = mounted->mnt_root;
		nd->flags |= LOOKUP_JUMPED;
		nd->seq = read_seqcount_begin(&path->dentry->d_seq);
		/*
		 * Update the inode too. We don't need to re-check the
		 * dentry sequence number here after this d_inode read,
		 * because a mount-point is always pinned.
		 */
		*inode = path->dentry->d_inode;
#ifdef MY_ABC_HERE
		nd->flags |= LOOKUP_MOUNTED;
#endif
	}
	return true;
}

static void follow_mount_rcu(struct nameidata *nd)
{
	while (d_mountpoint(nd->path.dentry)) {
		struct vfsmount *mounted;
		mounted = __lookup_mnt(nd->path.mnt, nd->path.dentry, 1);
		if (!mounted)
			break;
		nd->path.mnt = mounted;
		nd->path.dentry = mounted->mnt_root;
		nd->seq = read_seqcount_begin(&nd->path.dentry->d_seq);
	}
}

static int follow_dotdot_rcu(struct nameidata *nd)
{
	set_root_rcu(nd);

	while (1) {
		if (nd->path.dentry == nd->root.dentry &&
		    nd->path.mnt == nd->root.mnt) {
			break;
		}
		if (nd->path.dentry != nd->path.mnt->mnt_root) {
			struct dentry *old = nd->path.dentry;
			struct dentry *parent = old->d_parent;
			unsigned seq;

			seq = read_seqcount_begin(&parent->d_seq);
			if (read_seqcount_retry(&old->d_seq, nd->seq))
				goto failed;
			nd->path.dentry = parent;
			nd->seq = seq;
			break;
		}
		if (!follow_up_rcu(&nd->path))
			break;
		nd->seq = read_seqcount_begin(&nd->path.dentry->d_seq);
	}
	follow_mount_rcu(nd);
	nd->inode = nd->path.dentry->d_inode;
	return 0;

failed:
	nd->flags &= ~LOOKUP_RCU;
	if (!(nd->flags & LOOKUP_ROOT))
		nd->root.mnt = NULL;
	rcu_read_unlock();
	br_read_unlock(vfsmount_lock);
	return -ECHILD;
}

/*
 * Follow down to the covering mount currently visible to userspace.  At each
 * point, the filesystem owning that dentry may be queried as to whether the
 * caller is permitted to proceed or not.
 */
int follow_down(struct path *path)
{
	unsigned managed;
	int ret;

	while (managed = ACCESS_ONCE(path->dentry->d_flags),
	       unlikely(managed & DCACHE_MANAGED_DENTRY)) {
		/* Allow the filesystem to manage the transit without i_mutex
		 * being held.
		 *
		 * We indicate to the filesystem if someone is trying to mount
		 * something here.  This gives autofs the chance to deny anyone
		 * other than its daemon the right to mount on its
		 * superstructure.
		 *
		 * The filesystem may sleep at this point.
		 */
		if (managed & DCACHE_MANAGE_TRANSIT) {
			BUG_ON(!path->dentry->d_op);
			BUG_ON(!path->dentry->d_op->d_manage);
			ret = path->dentry->d_op->d_manage(
				path->dentry, false);
			if (ret < 0)
				return ret == -EISDIR ? 0 : ret;
		}

		/* Transit to a mounted filesystem. */
		if (managed & DCACHE_MOUNTED) {
			struct vfsmount *mounted = lookup_mnt(path);
			if (!mounted)
				break;
			dput(path->dentry);
			mntput(path->mnt);
			path->mnt = mounted;
			path->dentry = dget(mounted->mnt_root);
			continue;
		}

		/* Don't handle automount points here */
		break;
	}
	return 0;
}

/*
 * Skip to top of mountpoint pile in refwalk mode for follow_dotdot()
 */
#ifdef MY_ABC_HERE
static void follow_mount(struct nameidata *nd)
#else
static void follow_mount(struct path *path)
#endif
{
#ifdef MY_ABC_HERE
	struct path *path = &nd->path;
#endif
	while (d_mountpoint(path->dentry)) {
		struct vfsmount *mounted = lookup_mnt(path);
		if (!mounted)
			break;
		dput(path->dentry);
		mntput(path->mnt);
		path->mnt = mounted;
		path->dentry = dget(mounted->mnt_root);
#ifdef MY_ABC_HERE
		nd->flags |= LOOKUP_MOUNTED;
#endif
	}
}

static void follow_dotdot(struct nameidata *nd)
{
	set_root(nd);

	while(1) {
		struct dentry *old = nd->path.dentry;

		if (nd->path.dentry == nd->root.dentry &&
		    nd->path.mnt == nd->root.mnt) {
			break;
		}
		if (nd->path.dentry != nd->path.mnt->mnt_root) {
			/* rare case of legitimate dget_parent()... */
			nd->path.dentry = dget_parent(nd->path.dentry);
			dput(old);
			break;
		}
		if (!follow_up(&nd->path))
			break;
	}
#ifdef MY_ABC_HERE
	follow_mount(nd);
#else
	follow_mount(&nd->path);
#endif
	nd->inode = nd->path.dentry->d_inode;
}

/*
 * Allocate a dentry with name and parent, and perform a parent
 * directory ->lookup on it. Returns the new dentry, or ERR_PTR
 * on error. parent->d_inode->i_mutex must be held. d_lookup must
 * have verified that no child exists while under i_mutex.
 */
static struct dentry *d_alloc_and_lookup(struct dentry *parent,
				struct qstr *name, struct nameidata *nd)
{
	struct inode *inode = parent->d_inode;
	struct dentry *dentry;
	struct dentry *old;

	/* Don't create child dentry for a dead directory. */
	if (unlikely(IS_DEADDIR(inode)))
		return ERR_PTR(-ENOENT);

	dentry = d_alloc(parent, name);
	if (unlikely(!dentry))
		return ERR_PTR(-ENOMEM);

	old = inode->i_op->lookup(inode, dentry, nd);
	if (unlikely(old)) {
		dput(dentry);
		dentry = old;
	}
	return dentry;
}

/*
 * We already have a dentry, but require a lookup to be performed on the parent
 * directory to fill in d_inode. Returns the new dentry, or ERR_PTR on error.
 * parent->d_inode->i_mutex must be held. d_lookup must have verified that no
 * child exists while under i_mutex.
 */
static struct dentry *d_inode_lookup(struct dentry *parent, struct dentry *dentry,
				     struct nameidata *nd)
{
	struct inode *inode = parent->d_inode;
	struct dentry *old;

	/* Don't create child dentry for a dead directory. */
	if (unlikely(IS_DEADDIR(inode))) {
		dput(dentry);
		return ERR_PTR(-ENOENT);
	}

	old = inode->i_op->lookup(inode, dentry, nd);
	if (unlikely(old)) {
		dput(dentry);
		dentry = old;
	}
	return dentry;
}

/*
 *  It's more convoluted than I'd like it to be, but... it's still fairly
 *  small and for now I'd prefer to have fast path as straight as possible.
 *  It _is_ time-critical.
 */
static int do_lookup(struct nameidata *nd, struct qstr *name,
			struct path *path, struct inode **inode)
{
	struct vfsmount *mnt = nd->path.mnt;
	struct dentry *dentry, *parent = nd->path.dentry;
	int need_reval = 1;
	int status = 1;
	int err;
#ifdef MY_ABC_HERE
	int caseless = (LOOKUP_CASELESS_COMPARE & nd->flags)?1:0;
#endif

	/*
	 * Rename seqlock is not required here because in the off chance
	 * of a false negative due to a concurrent rename, we're going to
	 * do the non-racy lookup, below.
	 */
	if (nd->flags & LOOKUP_RCU) {
		unsigned seq;
		*inode = nd->inode;
#ifdef MY_ABC_HERE
		dentry = __d_lookup_rcu(parent, name, &seq, inode, caseless);
#else
		dentry = __d_lookup_rcu(parent, name, &seq, inode);
#endif
		if (!dentry)
			goto unlazy;

		/* Memory barrier in read_seqcount_begin of child is enough */
		if (__read_seqcount_retry(&parent->d_seq, nd->seq))
			return -ECHILD;
		nd->seq = seq;

		if (unlikely(dentry->d_flags & DCACHE_OP_REVALIDATE)) {
			status = d_revalidate(dentry, nd);
			if (unlikely(status <= 0)) {
				if (status != -ECHILD)
					need_reval = 0;
				goto unlazy;
			}
		}
		if (unlikely(d_need_lookup(dentry)))
			goto unlazy;
		path->mnt = mnt;
		path->dentry = dentry;
		if (unlikely(!__follow_mount_rcu(nd, path, inode)))
			goto unlazy;
		if (unlikely(path->dentry->d_flags & DCACHE_NEED_AUTOMOUNT))
			goto unlazy;
		return 0;
unlazy:
		if (unlazy_walk(nd, dentry))
			return -ECHILD;
	} else {
#ifdef MY_ABC_HERE
		dentry = __d_lookup(parent, name, caseless);
		if (caseless) {
			if (dentry && !dentry->d_inode) {
				struct inode *dir = parent->d_inode;

				d_invalidate(dentry);
				dput(dentry);
				dentry = NULL;

				mutex_lock(&dir->i_mutex);
				dentry = d_alloc_and_lookup(parent, name, nd);
				mutex_unlock(&dir->i_mutex);
				if (IS_ERR(dentry)) {
					dentry = NULL;
				}
			}
		}
#else
		dentry = __d_lookup(parent, name);
#endif
	}

	if (dentry && unlikely(d_need_lookup(dentry))) {
		dput(dentry);
		dentry = NULL;
	}
retry:
	if (unlikely(!dentry)) {
		struct inode *dir = parent->d_inode;
		BUG_ON(nd->inode != dir);

		mutex_lock(&dir->i_mutex);
#ifdef MY_ABC_HERE
		dentry = d_lookup_case(parent, name, caseless);
#else
		dentry = d_lookup(parent, name);
#endif
		if (likely(!dentry)) {
			dentry = d_alloc_and_lookup(parent, name, nd);
			if (IS_ERR(dentry)) {
				mutex_unlock(&dir->i_mutex);
				return PTR_ERR(dentry);
			}
			/* known good */
			need_reval = 0;
			status = 1;
		} else if (unlikely(d_need_lookup(dentry))) {
			dentry = d_inode_lookup(parent, dentry, nd);
			if (IS_ERR(dentry)) {
				mutex_unlock(&dir->i_mutex);
				return PTR_ERR(dentry);
			}
			/* known good */
			need_reval = 0;
			status = 1;
		}
		mutex_unlock(&dir->i_mutex);
	}
	if (unlikely(dentry->d_flags & DCACHE_OP_REVALIDATE) && need_reval)
		status = d_revalidate(dentry, nd);
	if (unlikely(status <= 0)) {
		if (status < 0) {
			dput(dentry);
			return status;
		}
		if (!d_invalidate(dentry)) {
			dput(dentry);
			dentry = NULL;
			need_reval = 1;
			goto retry;
		}
	}

	path->mnt = mnt;
	path->dentry = dentry;
#ifdef MY_ABC_HERE
	err = follow_managed(path, nd);
#else
	err = follow_managed(path, nd->flags);
#endif
	if (unlikely(err < 0)) {
		path_put_conditional(path, nd);
		return err;
	}
	if (err)
		nd->flags |= LOOKUP_JUMPED;
	*inode = path->dentry->d_inode;
	return 0;
}

static inline int may_lookup(struct nameidata *nd)
{
#ifdef CONFIG_FS_SYNO_ACL
	int err;
	int is_synoacl = IS_SYNOACL_INODE(nd->inode, nd->path.dentry);
#endif

	if (nd->flags & LOOKUP_RCU) {
#ifdef CONFIG_FS_SYNO_ACL
		if (is_synoacl) {
			err = synoacl_op_exec_perm(nd->path.dentry, nd->inode);
		} else {
			err = inode_permission(nd->inode, MAY_EXEC|MAY_NOT_BLOCK);
		}
#else
		int err = inode_permission(nd->inode, MAY_EXEC|MAY_NOT_BLOCK);
#endif
		if (err != -ECHILD)
			return err;
		if (unlazy_walk(nd, NULL))
			return -ECHILD;
	}

#ifdef CONFIG_FS_SYNO_ACL
	if (is_synoacl) {
		err = synoacl_op_exec_perm(nd->path.dentry, nd->inode);
	} else {
		err = inode_permission(nd->inode, MAY_EXEC);
	}
	return err;
#else
	return inode_permission(nd->inode, MAY_EXEC);
#endif
}

static inline int handle_dots(struct nameidata *nd, int type)
{
	if (type == LAST_DOTDOT) {
		if (nd->flags & LOOKUP_RCU) {
			if (follow_dotdot_rcu(nd))
				return -ECHILD;
		} else
			follow_dotdot(nd);
	}
	return 0;
}

static void terminate_walk(struct nameidata *nd)
{
	if (!(nd->flags & LOOKUP_RCU)) {
		path_put(&nd->path);
	} else {
		nd->flags &= ~LOOKUP_RCU;
		if (!(nd->flags & LOOKUP_ROOT))
			nd->root.mnt = NULL;
		rcu_read_unlock();
		br_read_unlock(vfsmount_lock);
	}
}

/*
 * Do we need to follow links? We _really_ want to be able
 * to do this check without having to look at inode->i_op,
 * so we keep a cache of "no, this doesn't need follow_link"
 * for the common case.
 */
static inline int should_follow_link(struct inode *inode, int follow)
{
	if (unlikely(!(inode->i_opflags & IOP_NOFOLLOW))) {
#ifdef MY_ABC_HERE
		if (likely(inode->i_op && inode->i_op->follow_link))
#else
		if (likely(inode->i_op->follow_link))
#endif
			return follow;

		/* This gets set once for the inode lifetime */
		spin_lock(&inode->i_lock);
		inode->i_opflags |= IOP_NOFOLLOW;
		spin_unlock(&inode->i_lock);
	}
	return 0;
}

#ifdef MY_ABC_HERE
static inline int update_real_filename(struct nameidata *nd, char *szTargetName, int targetLen)
{
	if ((nd->real_filename_len + targetLen + 2) >= SYNO_SMB_PSTRING_LEN) {
		return -1;
	}
	memcpy(nd->real_filename_cur_locate, szTargetName, targetLen);
	nd->real_filename_cur_locate += targetLen;
	nd->real_filename_len += targetLen;
	if (!(nd->flags & LOOKUP_TO_LASTCOMPONENT)) {
		*(nd->real_filename_cur_locate) = '/';
		nd->real_filename_cur_locate++;
		nd->real_filename_len ++;
	}
	*(nd->real_filename_cur_locate) = '\0';
	return 0;
}
#endif

static inline int walk_component(struct nameidata *nd, struct path *path,
		struct qstr *name, int type, int follow)
{
	struct inode *inode;
	int err;
	/*
	 * "." and ".." are special - ".." especially so because it has
	 * to be able to know about the current root directory and
	 * parent relationships.
	 */
	if (unlikely(type != LAST_NORM))
		return handle_dots(nd, type);
	err = do_lookup(nd, name, path, &inode);
	if (unlikely(err)) {
		terminate_walk(nd);
		return err;
	}
#ifdef MY_ABC_HERE
	/* After __follow_mount, next.dentry->d_name.name may be replaced to mnt path .
	 * The original path name is in next.mnt and it is what we need.
	*/
	if (LOOKUP_CASELESS_COMPARE & nd->flags) {
		int   targetLen = 0;
		char *szTargetName = NULL;

		if (nd->flags & LOOKUP_MOUNTED) {
			nd->flags &= ~LOOKUP_MOUNTED;
			szTargetName = (char *)path->mnt->mnt_mountpoint->d_name.name;
			targetLen = path->mnt->mnt_mountpoint->d_name.len;
		} else {
			szTargetName = (char *)path->dentry->d_name.name;
			targetLen = path->dentry->d_name.len;
		}
		if (update_real_filename(nd, szTargetName, targetLen)) {
			terminate_walk(nd);
			return -ENAMETOOLONG;
		}
	}
#endif
	if (!inode) {
		path_to_nameidata(path, nd);
		terminate_walk(nd);
		return -ENOENT;
	}
	if (should_follow_link(inode, follow)) {
		if (nd->flags & LOOKUP_RCU) {
			if (unlikely(unlazy_walk(nd, path->dentry))) {
				terminate_walk(nd);
				return -ECHILD;
			}
		}
		BUG_ON(inode != path->dentry->d_inode);
		return 1;
	}
	path_to_nameidata(path, nd);
	nd->inode = inode;
	return 0;
}

/*
 * This limits recursive symlink follows to 8, while
 * limiting consecutive symlinks to 40.
 *
 * Without that kind of total limit, nasty chains of consecutive
 * symlinks can cause almost arbitrarily long lookups.
 */
static inline int nested_symlink(struct path *path, struct nameidata *nd)
{
	int res;
#ifdef MY_ABC_HERE
	int blCaselessSet = 0;
#endif

	if (unlikely(current->link_count >= MAX_NESTED_LINKS)) {
		path_put_conditional(path, nd);
		path_put(&nd->path);
		return -ELOOP;
	}
	BUG_ON(nd->depth >= MAX_NESTED_LINKS);

	nd->depth++;
	current->link_count++;

#ifdef MY_ABC_HERE
	if (LOOKUP_CASELESS_COMPARE & nd->flags) {
		blCaselessSet = 1;
		nd->flags &= ~LOOKUP_CASELESS_COMPARE;
	}
#endif
	do {
		struct path link = *path;
		void *cookie;

		res = follow_link(&link, nd, &cookie);
		if (!res)
			res = walk_component(nd, path, &nd->last,
								 nd->last_type, LOOKUP_FOLLOW);
		put_link(nd, &link, cookie);
	} while (res > 0);
#ifdef MY_ABC_HERE
	if (blCaselessSet) {
		nd->flags |= LOOKUP_CASELESS_COMPARE;
	}
#endif

	current->link_count--;
	nd->depth--;
	return res;
}

/*
 * We really don't want to look at inode->i_op->lookup
 * when we don't have to. So we keep a cache bit in
 * the inode ->i_opflags field that says "yes, we can
 * do lookup on this inode".
 */
static inline int can_lookup(struct inode *inode)
{
	if (likely(inode->i_opflags & IOP_LOOKUP))
		return 1;
	if (likely(!inode->i_op->lookup))
		return 0;

	/* We do this once for the lifetime of the inode */
	spin_lock(&inode->i_lock);
	inode->i_opflags |= IOP_LOOKUP;
	spin_unlock(&inode->i_lock);
	return 1;
}

/*
 * Name resolution.
 * This is the basic name resolution function, turning a pathname into
 * the final dentry. We expect 'base' to be positive and a directory.
 *
 * Returns 0 and nd will have valid dentry and mnt on success.
 * Returns error and drops reference to input namei data on failure.
 */
static int link_path_walk(const char *name, struct nameidata *nd)
{
	struct path next;
	int err;
	
#ifdef MY_ABC_HERE
	/* We do case conversions here.
	 * The filename converted will be stored in nd->real_filename.
	 *
	 * In ext3_find_entry (ext3_dx_find_entry and search_dirblock),
	 * we sync file name of dentry stored in dentry queue with filename founded from disk.
	 * So the filename of dentry returned by do_lookup is case converted.
	 *
	 * Note:
	 * 1. If stat success, it will be copied to user space.
	 *    It means if any error occurs, we don't need store anything in nd->real_filename.
	 * 2. We should correctly update "name" and "slashes" to "cur_location" each loop.
	 * 	  We update them in every "continue", "break", "return" point.
	 * 3. If converted string longer than SYNO_SMB_PSTRING_LEN, we should return ENAMETOOLONG.
	 * 	  We use total_len to monitor it.
	*/
	int caselessFlag = LOOKUP_CASELESS_COMPARE & nd->flags;
	struct qstr this;

	while (*name=='/') {
		if (caselessFlag) {
			if (update_real_filename(nd, (char *) "", 0)) {
				terminate_walk(nd);
				return -ENAMETOOLONG;
			}
		}
		name++;
	}
#else
	while (*name=='/')
		name++;
#endif
	if (!*name)
		return 0;

	/* At this point we know we have a real path component. */
	for(;;) {
		unsigned long hash;
#ifndef MY_ABC_HERE
		struct qstr this;
#endif
		unsigned int c;
		int type;

#ifdef MY_ABC_HERE
		int slashCount = 0;
		nd->flags &= ~LOOKUP_MOUNTED;
#endif

		err = may_lookup(nd);
		if (err)
			break;

		this.name = name;
		c = *(const unsigned char *)name;

		hash = init_name_hash();
		do {
			name++;
			hash = partial_name_hash(c, hash);
			c = *(const unsigned char *)name;
		} while (c && (c != '/'));
		this.len = name - (const char *) this.name;
		this.hash = end_name_hash(hash);

		type = LAST_NORM;
		if (this.name[0] == '.') switch (this.len) {
			case 2:
				if (this.name[1] == '.') {
					type = LAST_DOTDOT;
					nd->flags |= LOOKUP_JUMPED;
				}
				break;
			case 1:
				type = LAST_DOT;
			}
		if (likely(type == LAST_NORM)) {
			struct dentry *parent = nd->path.dentry;
			nd->flags &= ~LOOKUP_JUMPED;
			if (unlikely(parent->d_flags & DCACHE_OP_HASH)) {
				err = parent->d_op->d_hash(parent, nd->inode,
										   &this);
				if (err < 0)
					break;
			}
		}

#ifdef MY_ABC_HERE
		if (caselessFlag && (LAST_DOTDOT == type || LAST_DOT == type)) {
			if (!c) nd->flags |= LOOKUP_TO_LASTCOMPONENT;
			if (update_real_filename(nd, (char *) this.name, this.len)) {
				terminate_walk(nd);
				return -ENAMETOOLONG;
			}
		}
#endif
		/* remove trailing slashes? */
		if (!c)
			goto last_component;
#ifdef MY_ABC_HERE
		while (*++name == '/') slashCount++;
#else
		while (*++name == '/');
#endif
		if (!*name)
			goto last_component;

		err = walk_component(nd, &next, &this, type, LOOKUP_FOLLOW);
		if (err < 0)
			return err;

#ifdef MY_ABC_HERE
		if (caselessFlag) {
			while (slashCount) {
				if (update_real_filename(nd, (char *) "", 0)) {
					terminate_walk(nd);
					return -ENAMETOOLONG;
				}
				slashCount--;
			}
		}
#endif
		if (err) {
			err = nested_symlink(&next, nd);
			if (err)
				return err;
		}
		if (can_lookup(nd->inode))
			continue;
		err = -ENOTDIR; 
		break;
		/* here ends the main loop */

last_component:
#ifdef MY_ABC_HERE
		nd->flags |= LOOKUP_TO_LASTCOMPONENT;
#endif
		nd->last = this;
		nd->last_type = type;
		return 0;
	}
	terminate_walk(nd);

	return err;
}

static int path_init(int dfd, const char *name, unsigned int flags,
		     struct nameidata *nd, struct file **fp)
{
	int retval = 0;
	int fput_needed;
	struct file *file;

	nd->last_type = LAST_ROOT; /* if there are only slashes... */
	nd->flags = flags | LOOKUP_JUMPED;
	nd->depth = 0;
	if (flags & LOOKUP_ROOT) {
		struct inode *inode = nd->root.dentry->d_inode;
		if (*name) {
			if (!inode->i_op->lookup)
				return -ENOTDIR;
			retval = inode_permission(inode, MAY_EXEC);
			if (retval)
				return retval;
		}
		nd->path = nd->root;
		nd->inode = inode;
		if (flags & LOOKUP_RCU) {
			br_read_lock(vfsmount_lock);
			rcu_read_lock();
			nd->seq = __read_seqcount_begin(&nd->path.dentry->d_seq);
		} else {
			path_get(&nd->path);
		}
		return 0;
	}

	nd->root.mnt = NULL;

	if (*name=='/') {
		if (flags & LOOKUP_RCU) {
			br_read_lock(vfsmount_lock);
			rcu_read_lock();
			set_root_rcu(nd);
		} else {
			set_root(nd);
			path_get(&nd->root);
		}
		nd->path = nd->root;
	} else if (dfd == AT_FDCWD) {
		if (flags & LOOKUP_RCU) {
			struct fs_struct *fs = current->fs;
			unsigned seq;

			br_read_lock(vfsmount_lock);
			rcu_read_lock();

			do {
				seq = read_seqcount_begin(&fs->seq);
				nd->path = fs->pwd;
				nd->seq = __read_seqcount_begin(&nd->path.dentry->d_seq);
			} while (read_seqcount_retry(&fs->seq, seq));
		} else {
			get_fs_pwd(current->fs, &nd->path);
		}
	} else {
		struct dentry *dentry;

		file = fget_raw_light(dfd, &fput_needed);
		retval = -EBADF;
		if (!file)
			goto out_fail;

		dentry = file->f_path.dentry;

		if (*name) {
			retval = -ENOTDIR;
			if (!S_ISDIR(dentry->d_inode->i_mode))
				goto fput_fail;

#ifdef CONFIG_FS_SYNO_ACL
			if (IS_SYNOACL(dentry)) {
				retval = synoacl_op_perm(dentry, MAY_EXEC);
			} else 
#endif
			retval = inode_permission(dentry->d_inode, MAY_EXEC);
			if (retval)
				goto fput_fail;
		}

		nd->path = file->f_path;
		if (flags & LOOKUP_RCU) {
			if (fput_needed)
				*fp = file;
			nd->seq = __read_seqcount_begin(&nd->path.dentry->d_seq);
			br_read_lock(vfsmount_lock);
			rcu_read_lock();
		} else {
			path_get(&file->f_path);
			fput_light(file, fput_needed);
		}
	}

	nd->inode = nd->path.dentry->d_inode;
	return 0;

fput_fail:
	fput_light(file, fput_needed);
out_fail:
	return retval;
}

static inline int lookup_last(struct nameidata *nd, struct path *path)
{
	if (nd->last_type == LAST_NORM && nd->last.name[nd->last.len])
		nd->flags |= LOOKUP_FOLLOW | LOOKUP_DIRECTORY;

#ifdef MY_ABC_HERE
	if (LOOKUP_CASELESS_COMPARE & nd->flags) {
		nd->flags &= ~LOOKUP_MOUNTED;
	}
#endif
	nd->flags &= ~LOOKUP_PARENT;
	return walk_component(nd, path, &nd->last, nd->last_type,
					nd->flags & LOOKUP_FOLLOW);
}

/* Returns 0 and nd will be valid on success; Retuns error, otherwise. */
static int path_lookupat(int dfd, const char *name,
				unsigned int flags, struct nameidata *nd)
{
	struct file *base = NULL;
	struct path path;
	int err;
#ifdef MY_ABC_HERE
	int blCaselessSet = 0;
#endif

	/*
	 * Path walking is largely split up into 2 different synchronisation
	 * schemes, rcu-walk and ref-walk (explained in
	 * Documentation/filesystems/path-lookup.txt). These share much of the
	 * path walk code, but some things particularly setup, cleanup, and
	 * following mounts are sufficiently divergent that functions are
	 * duplicated. Typically there is a function foo(), and its RCU
	 * analogue, foo_rcu().
	 *
	 * -ECHILD is the error number of choice (just to avoid clashes) that
	 * is returned if some aspect of an rcu-walk fails. Such an error must
	 * be handled by restarting a traditional ref-walk (which will always
	 * be able to complete).
	 */
	err = path_init(dfd, name, flags | LOOKUP_PARENT, nd, &base);

	if (unlikely(err))
		return err;

	current->total_link_count = 0;
	err = link_path_walk(name, nd);

	if (!err && !(flags & LOOKUP_PARENT)) {
		err = lookup_last(nd, &path);
#ifdef MY_ABC_HERE
		if (LOOKUP_CASELESS_COMPARE & nd->flags) {
			blCaselessSet = 1;
			nd->flags &= ~LOOKUP_CASELESS_COMPARE;
		}
#endif
		while (err > 0) {
			void *cookie;
			struct path link = path;
			nd->flags |= LOOKUP_PARENT;
			err = follow_link(&link, nd, &cookie);
			if (!err)
				err = lookup_last(nd, &path);
			put_link(nd, &link, cookie);
		}
#ifdef MY_ABC_HERE
		if (blCaselessSet) {
			nd->flags |= LOOKUP_CASELESS_COMPARE;
		}
#endif
	}

	if (!err)
		err = complete_walk(nd);

	if (!err && nd->flags & LOOKUP_DIRECTORY) {
		if (!nd->inode->i_op->lookup) {
			path_put(&nd->path);
			err = -ENOTDIR;
		}
	}

	if (base)
		fput(base);

	if (nd->root.mnt && !(nd->flags & LOOKUP_ROOT)) {
		path_put(&nd->root);
		nd->root.mnt = NULL;
	}
	return err;
}

static int do_path_lookup(int dfd, const char *name,
				unsigned int flags, struct nameidata *nd)
{
#ifdef MY_ABC_HERE
	int retval = path_lookupat(dfd, name, flags, nd);
#else
	int retval = path_lookupat(dfd, name, flags | LOOKUP_RCU, nd);
	if (unlikely(retval == -ECHILD))
		retval = path_lookupat(dfd, name, flags, nd);
#endif
	if (unlikely(retval == -ESTALE))
		retval = path_lookupat(dfd, name, flags | LOOKUP_REVAL, nd);

	if (likely(!retval)) {
		if (unlikely(!audit_dummy_context())) {
			if (nd->path.dentry && nd->inode)
				audit_inode(name, nd->path.dentry);
		}
	}
	return retval;
}

int kern_path_parent(const char *name, struct nameidata *nd)
{
	return do_path_lookup(AT_FDCWD, name, LOOKUP_PARENT, nd);
}

int kern_path(const char *name, unsigned int flags, struct path *path)
{
	struct nameidata nd;
	int res = do_path_lookup(AT_FDCWD, name, flags, &nd);
	if (!res)
		*path = nd.path;
	return res;
}

/**
 * vfs_path_lookup - lookup a file path relative to a dentry-vfsmount pair
 * @dentry:  pointer to dentry of the base directory
 * @mnt: pointer to vfs mount of the base directory
 * @name: pointer to file name
 * @flags: lookup flags
 * @path: pointer to struct path to fill
 */
int vfs_path_lookup(struct dentry *dentry, struct vfsmount *mnt,
		    const char *name, unsigned int flags,
		    struct path *path)
{
	struct nameidata nd;
	int err;
	nd.root.dentry = dentry;
	nd.root.mnt = mnt;
	BUG_ON(flags & LOOKUP_PARENT);
	/* the first argument of do_path_lookup() is ignored with LOOKUP_ROOT */
	err = do_path_lookup(AT_FDCWD, name, flags | LOOKUP_ROOT, &nd);
	if (!err)
		*path = nd.path;
	return err;
}

static struct dentry *__lookup_hash(struct qstr *name,
		struct dentry *base, struct nameidata *nd)
{
	struct inode *inode = base->d_inode;
	struct dentry *dentry;
	int err;

#ifdef CONFIG_FS_SYNO_ACL
	if (IS_SYNOACL(base)) {
		err = synoacl_op_exec_perm(base, inode);
	} else {
		err = inode_permission(inode, MAY_EXEC);
	}
#else
	err = inode_permission(inode, MAY_EXEC);
#endif

	if (err)
		return ERR_PTR(err);
	/*
	 * Don't bother with __d_lookup: callers are for creat as
	 * well as unlink, so a lot of the time it would cost
	 * a double lookup.
	 */
#ifdef MY_ABC_HERE
	dentry = d_lookup_case(base, name, (nd && (LOOKUP_CASELESS_COMPARE & nd->flags))?1:0);
#else
	dentry = d_lookup(base, name);
#endif

	if (dentry && d_need_lookup(dentry)) {
		/*
		 * __lookup_hash is called with the parent dir's i_mutex already
		 * held, so we are good to go here.
		 */
		dentry = d_inode_lookup(base, dentry, nd);
		if (IS_ERR(dentry))
			return dentry;
	}

	if (dentry && (dentry->d_flags & DCACHE_OP_REVALIDATE)) {
		int status = d_revalidate(dentry, nd);
		if (unlikely(status <= 0)) {
			/*
			 * The dentry failed validation.
			 * If d_revalidate returned 0 attempt to invalidate
			 * the dentry otherwise d_revalidate is asking us
			 * to return a fail status.
			 */
			if (status < 0) {
				dput(dentry);
				return ERR_PTR(status);
			} else if (!d_invalidate(dentry)) {
				dput(dentry);
				dentry = NULL;
			}
		}
	}

	if (!dentry)
		dentry = d_alloc_and_lookup(base, name, nd);

	return dentry;
}

/*
 * Restricted form of lookup. Doesn't follow links, single-component only,
 * needs parent already locked. Doesn't follow mounts.
 * SMP-safe.
 */
#ifdef CONFIG_AUFS_FS
extern struct dentry *lookup_hash(struct nameidata *nd)
#else /* !SYNO_AUFS */
static struct dentry *lookup_hash(struct nameidata *nd)
#endif /* SYNO_AUFS */
{
	return __lookup_hash(&nd->last, nd->path.dentry, nd);
}

/**
 * lookup_one_len - filesystem helper to lookup single pathname component
 * @name:	pathname component to lookup
 * @base:	base directory to lookup from
 * @len:	maximum length @len should be interpreted to
 *
 * Note that this routine is purely a helper for filesystem usage and should
 * not be called by generic code.  Also note that by using this function the
 * nameidata argument is passed to the filesystem methods and a filesystem
 * using this helper needs to be prepared for that.
 */
struct dentry *lookup_one_len(const char *name, struct dentry *base, int len)
{
	struct qstr this;
	unsigned long hash;
	unsigned int c;

	WARN_ON_ONCE(!mutex_is_locked(&base->d_inode->i_mutex));

	this.name = name;
	this.len = len;
	if (!len)
		return ERR_PTR(-EACCES);

	hash = init_name_hash();
	while (len--) {
		c = *(const unsigned char *)name++;
		if (c == '/' || c == '\0')
			return ERR_PTR(-EACCES);
		hash = partial_name_hash(c, hash);
	}
	this.hash = end_name_hash(hash);
	/*
	 * See if the low-level filesystem might want
	 * to use its own hash..
	 */
	if (base->d_flags & DCACHE_OP_HASH) {
		int err = base->d_op->d_hash(base, base->d_inode, &this);
		if (err < 0)
			return ERR_PTR(err);
	}

	return __lookup_hash(&this, base, NULL);
}

#ifdef MY_ABC_HERE
int syno_user_path_at(int dfd, const char __user *name, unsigned flags,
		 struct path *path, char **real_filename, int *real_filename_len, int *lastComponent)
{
	struct nameidata nd;
	char *tmp = getname(name);
	int err = PTR_ERR(tmp);
	if (!IS_ERR(tmp)) {

		BUG_ON(flags & LOOKUP_PARENT);

		nd.real_filename = *real_filename;
		nd.real_filename_cur_locate = nd.real_filename;
		nd.real_filename_len = 0;
		err = do_path_lookup(dfd, tmp, flags, &nd);
		putname(tmp);
		if (!err) {
			*path = nd.path;
			*real_filename_len = nd.real_filename_len;
			if (nd.flags & LOOKUP_TO_LASTCOMPONENT) {
				*lastComponent = 1;
			}
		}
	}
	return err;
}
#endif

int user_path_at_empty(int dfd, const char __user *name, unsigned flags,
		 struct path *path, int *empty)
{
	struct nameidata nd;
	char *tmp = getname_flags(name, flags, empty);
	int err = PTR_ERR(tmp);
	if (!IS_ERR(tmp)) {

		BUG_ON(flags & LOOKUP_PARENT);

		err = do_path_lookup(dfd, tmp, flags, &nd);
		putname(tmp);
		if (!err)
			*path = nd.path;
	}
	return err;
}

int user_path_at(int dfd, const char __user *name, unsigned flags,
		 struct path *path)
{
	return user_path_at_empty(dfd, name, flags, path, 0);
}

static int user_path_parent(int dfd, const char __user *path,
			struct nameidata *nd, char **name)
{
	char *s = getname(path);
	int error;

	if (IS_ERR(s))
		return PTR_ERR(s);

	error = do_path_lookup(dfd, s, LOOKUP_PARENT, nd);
	if (error)
		putname(s);
	else
		*name = s;

	return error;
}

/*
 * It's inline, so penalty for filesystems that don't use sticky bit is
 * minimal.
 */
static inline int check_sticky(struct inode *dir, struct inode *inode)
{
	uid_t fsuid = current_fsuid();

	if (!(dir->i_mode & S_ISVTX))
		return 0;
	if (current_user_ns() != inode_userns(inode))
		goto other_userns;
	if (inode->i_uid == fsuid)
		return 0;
	if (dir->i_uid == fsuid)
		return 0;

other_userns:
	return !ns_capable(inode_userns(inode), CAP_FOWNER);
}

/*
 *	Check whether we can remove a link victim from directory dir, check
 *  whether the type of victim is right.
 *  1. We can't do it if dir is read-only (done in permission())
 *  2. We should have write and exec permissions on dir
 *  3. We can't remove anything from append-only dir
 *  4. We can't do anything with immutable dir (done in permission())
 *  5. If the sticky bit on dir is set we should either
 *	a. be owner of dir, or
 *	b. be owner of victim, or
 *	c. have CAP_FOWNER capability
 *  6. If the victim is append-only or immutable we can't do antyhing with
 *     links pointing to it.
 *  7. If we were asked to remove a directory and victim isn't one - ENOTDIR.
 *  8. If we were asked to remove a non-directory and victim isn't one - EISDIR.
 *  9. We can't remove a root or mountpoint.
 * 10. We don't allow removal of NFS sillyrenamed files; it's handled by
 *     nfs_async_unlink().
 */
static int may_delete(struct inode *dir,struct dentry *victim,int isdir)
{
	int error;

	if (!victim->d_inode)
		return -ENOENT;

	BUG_ON(victim->d_parent->d_inode != dir);
	audit_inode_child(victim, dir);

#ifdef CONFIG_FS_SYNO_ACL
	if (IS_FS_SYNOACL(dir)) {
		error = synoacl_mod_may_delete(victim, dir);
	} else
#endif
	error = inode_permission(dir, MAY_WRITE | MAY_EXEC);
	if (error)
		return error;
	if (IS_APPEND(dir))
		return -EPERM;
#ifdef CONFIG_FS_SYNO_ACL
	if (!IS_SYNOACL(victim->d_parent) && check_sticky(dir, victim->d_inode)) {
		return -EPERM;
	}
	if (IS_APPEND(victim->d_inode) || IS_IMMUTABLE(victim->d_inode) || IS_SWAPFILE(victim->d_inode)){
		return -EPERM;
	}
#else
	if (check_sticky(dir, victim->d_inode)||IS_APPEND(victim->d_inode)||
	    IS_IMMUTABLE(victim->d_inode) || IS_SWAPFILE(victim->d_inode))
		return -EPERM;
#endif /* CONFIG_FS_SYNO_ACL */
	if (isdir) {
		if (!S_ISDIR(victim->d_inode->i_mode))
			return -ENOTDIR;
		if (IS_ROOT(victim))
			return -EBUSY;
	} else if (S_ISDIR(victim->d_inode->i_mode))
		return -EISDIR;
	if (IS_DEADDIR(dir))
		return -ENOENT;
	if (victim->d_flags & DCACHE_NFSFS_RENAMED)
		return -EBUSY;
	return 0;
}

/*	Check whether we can create an object with dentry child in directory
 *  dir.
 *  1. We can't do it if child already exists (open has special treatment for
 *     this case, but since we are inlined it's OK)
 *  2. We can't do it if dir is read-only (done in permission())
 *  3. We should have write and exec permissions on dir
 *  4. We can't do it if dir is immutable (done in permission())
 */
#ifdef CONFIG_FS_SYNO_ACL
static inline int may_create(struct inode *dir, struct dentry *child, int mode)
#else 
static inline int may_create(struct inode *dir, struct dentry *child)
#endif
{
	if (child->d_inode)
		return -EEXIST;
	if (IS_DEADDIR(dir))
		return -ENOENT;

#ifdef CONFIG_FS_SYNO_ACL
	if (IS_SYNOACL(child->d_parent)) {
		return synoacl_op_perm(child->d_parent, (S_ISDIR(mode)?MAY_APPEND:MAY_WRITE) | MAY_EXEC);
	} 
#endif /* CONFIG_FS_SYNO_ACL */
	return inode_permission(dir, MAY_WRITE | MAY_EXEC);
}

/*
 * p1 and p2 should be directories on the same fs.
 */
struct dentry *lock_rename(struct dentry *p1, struct dentry *p2)
{
	struct dentry *p;

	if (p1 == p2) {
		mutex_lock_nested(&p1->d_inode->i_mutex, I_MUTEX_PARENT);
		return NULL;
	}

	mutex_lock(&p1->d_inode->i_sb->s_vfs_rename_mutex);

	p = d_ancestor(p2, p1);
	if (p) {
		mutex_lock_nested(&p2->d_inode->i_mutex, I_MUTEX_PARENT);
		mutex_lock_nested(&p1->d_inode->i_mutex, I_MUTEX_CHILD);
		return p;
	}

	p = d_ancestor(p1, p2);
	if (p) {
		mutex_lock_nested(&p1->d_inode->i_mutex, I_MUTEX_PARENT);
		mutex_lock_nested(&p2->d_inode->i_mutex, I_MUTEX_CHILD);
		return p;
	}

	mutex_lock_nested(&p1->d_inode->i_mutex, I_MUTEX_PARENT);
	mutex_lock_nested(&p2->d_inode->i_mutex, I_MUTEX_CHILD);
	return NULL;
}

void unlock_rename(struct dentry *p1, struct dentry *p2)
{
	mutex_unlock(&p1->d_inode->i_mutex);
	if (p1 != p2) {
		mutex_unlock(&p2->d_inode->i_mutex);
		mutex_unlock(&p1->d_inode->i_sb->s_vfs_rename_mutex);
	}
}

int vfs_create(struct inode *dir, struct dentry *dentry, umode_t mode,
		struct nameidata *nd)
{
#ifdef CONFIG_FS_SYNO_ACL
	int error = may_create(dir, dentry, S_IFREG);
#else
	int error = may_create(dir, dentry);
#endif

	if (error)
		return error;

	if (!dir->i_op->create)
		return -EACCES;	/* shouldn't it be ENOSYS? */
	mode &= S_IALLUGO;
	mode |= S_IFREG;
	error = security_inode_create(dir, dentry, mode);
	if (error)
		return error;
	error = dir->i_op->create(dir, dentry, mode, nd);
	if (!error)
		fsnotify_create(dir, dentry);

#ifdef CONFIG_FS_SYNO_ACL
	if (!error && IS_SYNOACL(dentry->d_parent)) {
		//We assume that inode has been attached to dentry by d_instantiate().
		synoacl_op_init(dentry);
	}
#endif

	return error;
}

static int may_open(struct path *path, int acc_mode, int flag)
{
	struct dentry *dentry = path->dentry;
	struct inode *inode = dentry->d_inode;
	int error;

	/* O_PATH? */
	if (!acc_mode)
		return 0;

	if (!inode)
		return -ENOENT;

	switch (inode->i_mode & S_IFMT) {
	case S_IFLNK:
		return -ELOOP;
	case S_IFDIR:
		if (acc_mode & MAY_WRITE)
			return -EISDIR;
		break;
	case S_IFBLK:
	case S_IFCHR:
		if (path->mnt->mnt_flags & MNT_NODEV)
			return -EACCES;
		/*FALLTHRU*/
	case S_IFIFO:
	case S_IFSOCK:
		flag &= ~O_TRUNC;
		break;
	}

#ifdef CONFIG_FS_SYNO_ACL
	if (IS_SYNOACL(dentry)) {
		error = synoacl_op_perm(dentry, acc_mode);
	} else
#endif /* CONFIG_FS_SYNO_ACL */
	error = inode_permission(inode, acc_mode);
	if (error)
		return error;

	/*
	 * An append-only file must be opened in append mode for writing.
	 */
	if (IS_APPEND(inode)) {
		if  ((flag & O_ACCMODE) != O_RDONLY && !(flag & O_APPEND))
			return -EPERM;
		if (flag & O_TRUNC)
			return -EPERM;
	}

	/* O_NOATIME can only be set by the owner or superuser */
	if (flag & O_NOATIME && !inode_owner_or_capable(inode))
		return -EPERM;

	return 0;
}

static int handle_truncate(struct file *filp)
{
	struct path *path = &filp->f_path;
	struct inode *inode = path->dentry->d_inode;
	int error = get_write_access(inode);
	if (error)
		return error;
	/*
	 * Refuse to truncate files with mandatory locks held on them.
	 */
	error = locks_verify_locked(inode);
	if (!error)
		error = security_path_truncate(path);
	if (!error) {
		error = do_truncate(path->dentry, 0,
				    ATTR_MTIME|ATTR_CTIME|ATTR_OPEN,
				    filp);
	}
	put_write_access(inode);
	return error;
}

static inline int open_to_namei_flags(int flag)
{
	if ((flag & O_ACCMODE) == 3)
		flag--;
	return flag;
}

/*
 * Handle the last step of open()
 */
static struct file *do_last(struct nameidata *nd, struct path *path,
			    const struct open_flags *op, const char *pathname)
{
	struct dentry *dir = nd->path.dentry;
	struct dentry *dentry;
	int open_flag = op->open_flag;
	int will_truncate = open_flag & O_TRUNC;
	int want_write = 0;
	int acc_mode = op->acc_mode;
	struct file *filp;
	int error;

	nd->flags &= ~LOOKUP_PARENT;
	nd->flags |= op->intent;

	switch (nd->last_type) {
	case LAST_DOTDOT:
	case LAST_DOT:
		error = handle_dots(nd, nd->last_type);
		if (error)
			return ERR_PTR(error);
		/* fallthrough */
	case LAST_ROOT:
		error = complete_walk(nd);
		if (error)
			return ERR_PTR(error);
		audit_inode(pathname, nd->path.dentry);
		if (open_flag & O_CREAT) {
			error = -EISDIR;
			goto exit;
		}
		goto ok;
	case LAST_BIND:
		error = complete_walk(nd);
		if (error)
			return ERR_PTR(error);
		audit_inode(pathname, dir);
		goto ok;
	}

	if (!(open_flag & O_CREAT)) {
		int symlink_ok = 0;
		if (nd->last.name[nd->last.len])
			nd->flags |= LOOKUP_FOLLOW | LOOKUP_DIRECTORY;
		if (open_flag & O_PATH && !(nd->flags & LOOKUP_FOLLOW))
			symlink_ok = 1;
		/* we _can_ be in RCU mode here */
		error = walk_component(nd, path, &nd->last, LAST_NORM,
					!symlink_ok);
		if (error < 0)
			return ERR_PTR(error);
		if (error) /* symlink */
			return NULL;
		/* sayonara */
		error = complete_walk(nd);
		if (error)
			return ERR_PTR(error);

		error = -ENOTDIR;
		if (nd->flags & LOOKUP_DIRECTORY) {
			if (!nd->inode->i_op->lookup)
				goto exit;
		}
		audit_inode(pathname, nd->path.dentry);
		goto ok;
	}

	/* create side of things */
	/*
	 * This will *only* deal with leaving RCU mode - LOOKUP_JUMPED has been
	 * cleared when we got to the last component we are about to look up
	 */
	error = complete_walk(nd);
	if (error)
		return ERR_PTR(error);

	audit_inode(pathname, dir);
	error = -EISDIR;
	/* trailing slashes? */
	if (nd->last.name[nd->last.len])
		goto exit;

	mutex_lock(&dir->d_inode->i_mutex);

	dentry = lookup_hash(nd);
	error = PTR_ERR(dentry);
	if (IS_ERR(dentry)) {
		mutex_unlock(&dir->d_inode->i_mutex);
		goto exit;
	}

	path->dentry = dentry;
	path->mnt = nd->path.mnt;

	/* Negative dentry, just create the file */
	if (!dentry->d_inode) {
		int mode = op->mode;
		if (!IS_POSIXACL(dir->d_inode))
			mode &= ~current_umask();
		/*
		 * This write is needed to ensure that a
		 * rw->ro transition does not occur between
		 * the time when the file is created and when
		 * a permanent write count is taken through
		 * the 'struct file' in nameidata_to_filp().
		 */
		error = mnt_want_write(nd->path.mnt);
		if (error)
			goto exit_mutex_unlock;
		want_write = 1;
		/* Don't check for write permission, don't truncate */
		open_flag &= ~O_TRUNC;
		will_truncate = 0;
		acc_mode = MAY_OPEN;
		error = security_path_mknod(&nd->path, dentry, mode, 0);
		if (error)
			goto exit_mutex_unlock;
		error = vfs_create(dir->d_inode, dentry, mode, nd);
		if (error)
			goto exit_mutex_unlock;
		mutex_unlock(&dir->d_inode->i_mutex);
		dput(nd->path.dentry);
		nd->path.dentry = dentry;
		goto common;
	}

	/*
	 * It already exists.
	 */
	mutex_unlock(&dir->d_inode->i_mutex);
	audit_inode(pathname, path->dentry);

	error = -EEXIST;
	if (open_flag & O_EXCL)
		goto exit_dput;

#ifdef MY_ABC_HERE
	error = follow_managed(path, nd);
#else
	error = follow_managed(path, nd->flags);
#endif
	if (error < 0)
		goto exit_dput;

	if (error)
		nd->flags |= LOOKUP_JUMPED;

	error = -ENOENT;
	if (!path->dentry->d_inode)
		goto exit_dput;

	if (path->dentry->d_inode->i_op->follow_link)
		return NULL;

	path_to_nameidata(path, nd);
	nd->inode = path->dentry->d_inode;
	/* Why this, you ask?  _Now_ we might have grown LOOKUP_JUMPED... */
	error = complete_walk(nd);
	if (error)
		return ERR_PTR(error);
	error = -EISDIR;
	if (S_ISDIR(nd->inode->i_mode))
		goto exit;
ok:
	if (!S_ISREG(nd->inode->i_mode))
		will_truncate = 0;

	if (will_truncate) {
		error = mnt_want_write(nd->path.mnt);
		if (error)
			goto exit;
		want_write = 1;
	}
common:
	error = may_open(&nd->path, acc_mode, open_flag);
	if (error)
		goto exit;
	filp = nameidata_to_filp(nd);
	if (!IS_ERR(filp)) {
		error = ima_file_check(filp, op->acc_mode);
		if (error) {
			fput(filp);
			filp = ERR_PTR(error);
		}
	}
	if (!IS_ERR(filp)) {
		if (will_truncate) {
			error = handle_truncate(filp);
			if (error) {
				fput(filp);
				filp = ERR_PTR(error);
			}
		}
	}
out:
	if (want_write)
		mnt_drop_write(nd->path.mnt);
	path_put(&nd->path);
	return filp;

exit_mutex_unlock:
	mutex_unlock(&dir->d_inode->i_mutex);
exit_dput:
	path_put_conditional(path, nd);
exit:
	filp = ERR_PTR(error);
	goto out;
}

static struct file *path_openat(int dfd, const char *pathname,
		struct nameidata *nd, const struct open_flags *op, int flags)
{
	struct file *base = NULL;
	struct file *filp;
	struct path path;
	int error;

	filp = get_empty_filp();
	if (!filp)
		return ERR_PTR(-ENFILE);

	filp->f_flags = op->open_flag;
	nd->intent.open.file = filp;
	nd->intent.open.flags = open_to_namei_flags(op->open_flag);
	nd->intent.open.create_mode = op->mode;

	error = path_init(dfd, pathname, flags | LOOKUP_PARENT, nd, &base);
	if (unlikely(error))
		goto out_filp;

	current->total_link_count = 0;
	error = link_path_walk(pathname, nd);
	if (unlikely(error))
		goto out_filp;

	filp = do_last(nd, &path, op, pathname);
	while (unlikely(!filp)) { /* trailing symlink */
		struct path link = path;
		void *cookie;
		if (!(nd->flags & LOOKUP_FOLLOW)) {
			path_put_conditional(&path, nd);
			path_put(&nd->path);
			filp = ERR_PTR(-ELOOP);
			break;
		}
		nd->flags |= LOOKUP_PARENT;
		nd->flags &= ~(LOOKUP_OPEN|LOOKUP_CREATE|LOOKUP_EXCL);
		error = follow_link(&link, nd, &cookie);
		if (unlikely(error))
			filp = ERR_PTR(error);
		else
			filp = do_last(nd, &path, op, pathname);
		put_link(nd, &link, cookie);
	}
out:
	if (nd->root.mnt && !(nd->flags & LOOKUP_ROOT))
		path_put(&nd->root);
	if (base)
		fput(base);
	release_open_intent(nd);
	return filp;

out_filp:
	filp = ERR_PTR(error);
	goto out;
}

struct file *do_filp_open(int dfd, const char *pathname,
		const struct open_flags *op, int flags)
{
	struct nameidata nd;
	struct file *filp;

#ifdef MY_ABC_HERE
	filp = path_openat(dfd, pathname, &nd, op, flags);
#else
	filp = path_openat(dfd, pathname, &nd, op, flags | LOOKUP_RCU);
	if (unlikely(filp == ERR_PTR(-ECHILD)))
		filp = path_openat(dfd, pathname, &nd, op, flags);
#endif
	if (unlikely(filp == ERR_PTR(-ESTALE)))
		filp = path_openat(dfd, pathname, &nd, op, flags | LOOKUP_REVAL);
	return filp;
}

struct file *do_file_open_root(struct dentry *dentry, struct vfsmount *mnt,
		const char *name, const struct open_flags *op, int flags)
{
	struct nameidata nd;
	struct file *file;

	nd.root.mnt = mnt;
	nd.root.dentry = dentry;

	flags |= LOOKUP_ROOT;

	if (dentry->d_inode->i_op->follow_link && op->intent & LOOKUP_OPEN)
		return ERR_PTR(-ELOOP);

#ifdef MY_ABC_HERE
	file = path_openat(-1, name, &nd, op, flags);
#else
	file = path_openat(-1, name, &nd, op, flags | LOOKUP_RCU);
	if (unlikely(file == ERR_PTR(-ECHILD)))
		file = path_openat(-1, name, &nd, op, flags);
#endif
	if (unlikely(file == ERR_PTR(-ESTALE)))
		file = path_openat(-1, name, &nd, op, flags | LOOKUP_REVAL);
	return file;
}

struct dentry *kern_path_create(int dfd, const char *pathname, struct path *path, int is_dir)
{
	struct dentry *dentry = ERR_PTR(-EEXIST);
	struct nameidata nd;
	int error = do_path_lookup(dfd, pathname, LOOKUP_PARENT, &nd);
	if (error)
		return ERR_PTR(error);

	/*
	 * Yucky last component or no last component at all?
	 * (foo/., foo/.., /////)
	 */
	if (nd.last_type != LAST_NORM)
		goto out;
	nd.flags &= ~LOOKUP_PARENT;
	nd.flags |= LOOKUP_CREATE | LOOKUP_EXCL;
	nd.intent.open.flags = O_EXCL;

	/*
	 * Do the final lookup.
	 */
	mutex_lock_nested(&nd.path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	dentry = lookup_hash(&nd);
	if (IS_ERR(dentry))
		goto fail;

	if (dentry->d_inode)
		goto eexist;
	/*
	 * Special case - lookup gave negative, but... we had foo/bar/
	 * From the vfs_mknod() POV we just have a negative dentry -
	 * all is fine. Let's be bastards - you had / on the end, you've
	 * been asking for (non-existent) directory. -ENOENT for you.
	 */
	if (unlikely(!is_dir && nd.last.name[nd.last.len])) {
		dput(dentry);
		dentry = ERR_PTR(-ENOENT);
		goto fail;
	}
	*path = nd.path;
	return dentry;
eexist:
	dput(dentry);
	dentry = ERR_PTR(-EEXIST);
fail:
	mutex_unlock(&nd.path.dentry->d_inode->i_mutex);
out:
	path_put(&nd.path);
	return dentry;
}
EXPORT_SYMBOL(kern_path_create);

struct dentry *user_path_create(int dfd, const char __user *pathname, struct path *path, int is_dir)
{
	char *tmp = getname(pathname);
	struct dentry *res;
	if (IS_ERR(tmp))
		return ERR_CAST(tmp);
	res = kern_path_create(dfd, tmp, path, is_dir);
	putname(tmp);
	return res;
}
EXPORT_SYMBOL(user_path_create);

int vfs_mknod(struct inode *dir, struct dentry *dentry, umode_t mode, dev_t dev)
{
#ifdef CONFIG_FS_SYNO_ACL
	int error = may_create(dir, dentry, mode);
#else
	int error = may_create(dir, dentry);
#endif

	if (error)
		return error;

	if ((S_ISCHR(mode) || S_ISBLK(mode)) &&
	    !ns_capable(inode_userns(dir), CAP_MKNOD))
		return -EPERM;

	if (!dir->i_op->mknod)
		return -EPERM;

	error = devcgroup_inode_mknod(mode, dev);
	if (error)
		return error;

	error = security_inode_mknod(dir, dentry, mode, dev);
	if (error)
		return error;

	error = dir->i_op->mknod(dir, dentry, mode, dev);
	if (!error)
		fsnotify_create(dir, dentry);
	return error;
}

static int may_mknod(mode_t mode)
{
	switch (mode & S_IFMT) {
	case S_IFREG:
	case S_IFCHR:
	case S_IFBLK:
	case S_IFIFO:
	case S_IFSOCK:
	case 0: /* zero mode translates to S_IFREG */
		return 0;
	case S_IFDIR:
		return -EPERM;
	default:
		return -EINVAL;
	}
}

SYSCALL_DEFINE4(mknodat, int, dfd, const char __user *, filename, int, mode,
		unsigned, dev)
{
	struct dentry *dentry;
	struct path path;
	int error;

	if (S_ISDIR(mode))
		return -EPERM;

	dentry = user_path_create(dfd, filename, &path, 0);
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

	if (!IS_POSIXACL(path.dentry->d_inode))
		mode &= ~current_umask();
	error = may_mknod(mode);
	if (error)
		goto out_dput;
	error = mnt_want_write(path.mnt);
	if (error)
		goto out_dput;
	error = security_path_mknod(&path, dentry, mode, dev);
	if (error)
		goto out_drop_write;
	switch (mode & S_IFMT) {
		case 0: case S_IFREG:
			error = vfs_create(path.dentry->d_inode,dentry,mode,NULL);
			break;
		case S_IFCHR: case S_IFBLK:
			error = vfs_mknod(path.dentry->d_inode,dentry,mode,
					new_decode_dev(dev));
			break;
		case S_IFIFO: case S_IFSOCK:
			error = vfs_mknod(path.dentry->d_inode,dentry,mode,0);
			break;
	}
out_drop_write:
	mnt_drop_write(path.mnt);
out_dput:
	dput(dentry);
	mutex_unlock(&path.dentry->d_inode->i_mutex);
	path_put(&path);

	return error;
}

SYSCALL_DEFINE3(mknod, const char __user *, filename, int, mode, unsigned, dev)
{
	return sys_mknodat(AT_FDCWD, filename, mode, dev);
}

int vfs_mkdir(struct inode *dir, struct dentry *dentry, umode_t mode)
{
#ifdef CONFIG_FS_SYNO_ACL
	int error = may_create(dir, dentry, S_IFDIR);
#else
	int error = may_create(dir, dentry);
#endif
	if (error)
		return error;

	if (!dir->i_op->mkdir)
		return -EPERM;

	mode &= (S_IRWXUGO|S_ISVTX);
	error = security_inode_mkdir(dir, dentry, mode);
	if (error)
		return error;

	error = dir->i_op->mkdir(dir, dentry, mode);
	if (!error)
		fsnotify_mkdir(dir, dentry);

#ifdef CONFIG_FS_SYNO_ACL
	if (!error && IS_SYNOACL(dentry->d_parent)) {
		//We assume that inode has been attached to dentry by d_instantiate().
		synoacl_op_init(dentry);
	}
#endif

	return error;
}

SYSCALL_DEFINE3(mkdirat, int, dfd, const char __user *, pathname, int, mode)
{
	struct dentry *dentry;
	struct path path;
	int error;

	dentry = user_path_create(dfd, pathname, &path, 1);
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

	if (!IS_POSIXACL(path.dentry->d_inode))
		mode &= ~current_umask();
	error = mnt_want_write(path.mnt);
	if (error)
		goto out_dput;
	error = security_path_mkdir(&path, dentry, mode);
	if (error)
		goto out_drop_write;
	error = vfs_mkdir(path.dentry->d_inode, dentry, mode);
out_drop_write:
	mnt_drop_write(path.mnt);
out_dput:
	dput(dentry);
	mutex_unlock(&path.dentry->d_inode->i_mutex);
	path_put(&path);
	return error;
}

SYSCALL_DEFINE2(mkdir, const char __user *, pathname, int, mode)
{
	return sys_mkdirat(AT_FDCWD, pathname, mode);
}

/*
 * The dentry_unhash() helper will try to drop the dentry early: we
 * should have a usage count of 2 if we're the only user of this
 * dentry, and if that is true (possibly after pruning the dcache),
 * then we drop the dentry now.
 *
 * A low-level filesystem can, if it choses, legally
 * do a
 *
 *	if (!d_unhashed(dentry))
 *		return -EBUSY;
 *
 * if it cannot handle the case of removing a directory
 * that is still in use by something else..
 */
void dentry_unhash(struct dentry *dentry)
{
	shrink_dcache_parent(dentry);
	spin_lock(&dentry->d_lock);
	if (dentry->d_count == 1)
		__d_drop(dentry);
	spin_unlock(&dentry->d_lock);
}

int vfs_rmdir(struct inode *dir, struct dentry *dentry)
{
	int error = may_delete(dir, dentry, 1);

	if (error)
		return error;

	if (!dir->i_op->rmdir)
		return -EPERM;

	dget(dentry);
	mutex_lock(&dentry->d_inode->i_mutex);

	error = -EBUSY;
	if (d_mountpoint(dentry))
		goto out;

	error = security_inode_rmdir(dir, dentry);
	if (error)
		goto out;

	shrink_dcache_parent(dentry);
	error = dir->i_op->rmdir(dir, dentry);
	if (error)
		goto out;

	dentry->d_inode->i_flags |= S_DEAD;
	dont_mount(dentry);

out:
	mutex_unlock(&dentry->d_inode->i_mutex);
	dput(dentry);
	if (!error)
		d_delete(dentry);
	return error;
}

static long do_rmdir(int dfd, const char __user *pathname)
{
	int error = 0;
	char * name;
	struct dentry *dentry;
	struct nameidata nd;

	error = user_path_parent(dfd, pathname, &nd, &name);
	if (error)
		return error;

	switch(nd.last_type) {
	case LAST_DOTDOT:
		error = -ENOTEMPTY;
		goto exit1;
	case LAST_DOT:
		error = -EINVAL;
		goto exit1;
	case LAST_ROOT:
		error = -EBUSY;
		goto exit1;
	}

	nd.flags &= ~LOOKUP_PARENT;

	mutex_lock_nested(&nd.path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	dentry = lookup_hash(&nd);
	error = PTR_ERR(dentry);
	if (IS_ERR(dentry))
		goto exit2;
	if (!dentry->d_inode) {
		error = -ENOENT;
		goto exit3;
	}
	error = mnt_want_write(nd.path.mnt);
	if (error)
		goto exit3;
	error = security_path_rmdir(&nd.path, dentry);
	if (error)
		goto exit4;
	error = vfs_rmdir(nd.path.dentry->d_inode, dentry);
exit4:
	mnt_drop_write(nd.path.mnt);
exit3:
	dput(dentry);
exit2:
	mutex_unlock(&nd.path.dentry->d_inode->i_mutex);
exit1:
	path_put(&nd.path);
	putname(name);
	return error;
}

SYSCALL_DEFINE1(rmdir, const char __user *, pathname)
{
	return do_rmdir(AT_FDCWD, pathname);
}

int vfs_unlink(struct inode *dir, struct dentry *dentry)
{
	int error = may_delete(dir, dentry, 0);

	if (error)
		return error;

	if (!dir->i_op->unlink)
		return -EPERM;

	mutex_lock(&dentry->d_inode->i_mutex);
	if (d_mountpoint(dentry))
		error = -EBUSY;
	else {
		error = security_inode_unlink(dir, dentry);
		if (!error) {
			error = dir->i_op->unlink(dir, dentry);
			if (!error)
				dont_mount(dentry);
		}
	}
	mutex_unlock(&dentry->d_inode->i_mutex);

	/* We don't d_delete() NFS sillyrenamed files--they still exist. */
	if (!error && !(dentry->d_flags & DCACHE_NFSFS_RENAMED)) {
		fsnotify_link_count(dentry->d_inode);
		d_delete(dentry);
	}

	return error;
}

/*
 * Make sure that the actual truncation of the file will occur outside its
 * directory's i_mutex.  Truncate can take a long time if there is a lot of
 * writeout happening, and we don't want to prevent access to the directory
 * while waiting on the I/O.
 */
static long do_unlinkat(int dfd, const char __user *pathname)
{
	int error;
	char *name;
	struct dentry *dentry;
	struct nameidata nd;
	struct inode *inode = NULL;

	error = user_path_parent(dfd, pathname, &nd, &name);
	if (error)
		return error;

	error = -EISDIR;
	if (nd.last_type != LAST_NORM)
		goto exit1;

	nd.flags &= ~LOOKUP_PARENT;

	mutex_lock_nested(&nd.path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	dentry = lookup_hash(&nd);
	error = PTR_ERR(dentry);
	if (!IS_ERR(dentry)) {
		/* Why not before? Because we want correct error value */
		if (nd.last.name[nd.last.len])
			goto slashes;
		inode = dentry->d_inode;
		if (!inode)
			goto slashes;
		ihold(inode);
		error = mnt_want_write(nd.path.mnt);
		if (error)
			goto exit2;
		error = security_path_unlink(&nd.path, dentry);
		if (error)
			goto exit3;
		error = vfs_unlink(nd.path.dentry->d_inode, dentry);
exit3:
		mnt_drop_write(nd.path.mnt);
	exit2:
		dput(dentry);
	}
	mutex_unlock(&nd.path.dentry->d_inode->i_mutex);
	if (inode)
		iput(inode);	/* truncate the inode here */
exit1:
	path_put(&nd.path);
	putname(name);
	return error;

slashes:
	error = !dentry->d_inode ? -ENOENT :
		S_ISDIR(dentry->d_inode->i_mode) ? -EISDIR : -ENOTDIR;
	goto exit2;
}

SYSCALL_DEFINE3(unlinkat, int, dfd, const char __user *, pathname, int, flag)
{
	if ((flag & ~AT_REMOVEDIR) != 0)
		return -EINVAL;

	if (flag & AT_REMOVEDIR)
		return do_rmdir(dfd, pathname);

	return do_unlinkat(dfd, pathname);
}

SYSCALL_DEFINE1(unlink, const char __user *, pathname)
{
	return do_unlinkat(AT_FDCWD, pathname);
}

int vfs_symlink(struct inode *dir, struct dentry *dentry, const char *oldname)
{
#ifdef CONFIG_FS_SYNO_ACL
	int error = may_create(dir, dentry, S_IFLNK);
#else
	int error = may_create(dir, dentry);
#endif

	if (error)
		return error;

	if (!dir->i_op->symlink)
		return -EPERM;

	error = security_inode_symlink(dir, dentry, oldname);
	if (error)
		return error;

	error = dir->i_op->symlink(dir, dentry, oldname);
	if (!error)
		fsnotify_create(dir, dentry);
	return error;
}

SYSCALL_DEFINE3(symlinkat, const char __user *, oldname,
		int, newdfd, const char __user *, newname)
{
	int error;
	char *from;
	struct dentry *dentry;
	struct path path;

	from = getname(oldname);
	if (IS_ERR(from))
		return PTR_ERR(from);

	dentry = user_path_create(newdfd, newname, &path, 0);
	error = PTR_ERR(dentry);
	if (IS_ERR(dentry))
		goto out_putname;

	error = mnt_want_write(path.mnt);
	if (error)
		goto out_dput;
	error = security_path_symlink(&path, dentry, from);
	if (error)
		goto out_drop_write;
	error = vfs_symlink(path.dentry->d_inode, dentry, from);
out_drop_write:
	mnt_drop_write(path.mnt);
out_dput:
	dput(dentry);
	mutex_unlock(&path.dentry->d_inode->i_mutex);
	path_put(&path);
out_putname:
	putname(from);
	return error;
}

SYSCALL_DEFINE2(symlink, const char __user *, oldname, const char __user *, newname)
{
	return sys_symlinkat(oldname, AT_FDCWD, newname);
}

int vfs_link(struct dentry *old_dentry, struct inode *dir, struct dentry *new_dentry)
{
	struct inode *inode = old_dentry->d_inode;
	int error;

	if (!inode)
		return -ENOENT;

#ifdef CONFIG_FS_SYNO_ACL
	error = may_create(dir, new_dentry, inode->i_mode);
#else
	error = may_create(dir, new_dentry);
#endif
	if (error)
		return error;

	if (dir->i_sb != inode->i_sb)
		return -EXDEV;

	/*
	 * A link to an append-only or immutable file cannot be created.
	 */
	if (IS_APPEND(inode) || IS_IMMUTABLE(inode))
		return -EPERM;
	if (!dir->i_op->link)
		return -EPERM;
	if (S_ISDIR(inode->i_mode))
		return -EPERM;

	error = security_inode_link(old_dentry, dir, new_dentry);
	if (error)
		return error;

	mutex_lock(&inode->i_mutex);
	/* Make sure we don't allow creating hardlink to an unlinked file */
	if (inode->i_nlink == 0)
		error =  -ENOENT;
	else
		error = dir->i_op->link(old_dentry, dir, new_dentry);
	mutex_unlock(&inode->i_mutex);
	if (!error)
		fsnotify_link(dir, inode, new_dentry);
	return error;
}

/*
 * Hardlinks are often used in delicate situations.  We avoid
 * security-related surprises by not following symlinks on the
 * newname.  --KAB
 *
 * We don't follow them on the oldname either to be compatible
 * with linux 2.0, and to avoid hard-linking to directories
 * and other special files.  --ADM
 */
SYSCALL_DEFINE5(linkat, int, olddfd, const char __user *, oldname,
		int, newdfd, const char __user *, newname, int, flags)
{
	struct dentry *new_dentry;
	struct path old_path, new_path;
	int how = 0;
	int error;

	if ((flags & ~(AT_SYMLINK_FOLLOW | AT_EMPTY_PATH)) != 0)
		return -EINVAL;
	/*
	 * To use null names we require CAP_DAC_READ_SEARCH
	 * This ensures that not everyone will be able to create
	 * handlink using the passed filedescriptor.
	 */
	if (flags & AT_EMPTY_PATH) {
		if (!capable(CAP_DAC_READ_SEARCH))
			return -ENOENT;
		how = LOOKUP_EMPTY;
	}

	if (flags & AT_SYMLINK_FOLLOW)
		how |= LOOKUP_FOLLOW;

	error = user_path_at(olddfd, oldname, how, &old_path);
	if (error)
		return error;

	new_dentry = user_path_create(newdfd, newname, &new_path, 0);
	error = PTR_ERR(new_dentry);
	if (IS_ERR(new_dentry))
		goto out;

	error = -EXDEV;
	if (old_path.mnt != new_path.mnt)
		goto out_dput;
	error = mnt_want_write(new_path.mnt);
	if (error)
		goto out_dput;
	error = security_path_link(old_path.dentry, &new_path, new_dentry);
	if (error)
		goto out_drop_write;
	error = vfs_link(old_path.dentry, new_path.dentry->d_inode, new_dentry);
out_drop_write:
	mnt_drop_write(new_path.mnt);
out_dput:
	dput(new_dentry);
	mutex_unlock(&new_path.dentry->d_inode->i_mutex);
	path_put(&new_path);
out:
	path_put(&old_path);

	return error;
}

SYSCALL_DEFINE2(link, const char __user *, oldname, const char __user *, newname)
{
	return sys_linkat(AT_FDCWD, oldname, AT_FDCWD, newname, 0);
}

/*
 * The worst of all namespace operations - renaming directory. "Perverted"
 * doesn't even start to describe it. Somebody in UCB had a heck of a trip...
 * Problems:
 *	a) we can get into loop creation. Check is done in is_subdir().
 *	b) race potential - two innocent renames can create a loop together.
 *	   That's where 4.4 screws up. Current fix: serialization on
 *	   sb->s_vfs_rename_mutex. We might be more accurate, but that's another
 *	   story.
 *	c) we have to lock _three_ objects - parents and victim (if it exists).
 *	   And that - after we got ->i_mutex on parents (until then we don't know
 *	   whether the target exists).  Solution: try to be smart with locking
 *	   order for inodes.  We rely on the fact that tree topology may change
 *	   only under ->s_vfs_rename_mutex _and_ that parent of the object we
 *	   move will be locked.  Thus we can rank directories by the tree
 *	   (ancestors first) and rank all non-directories after them.
 *	   That works since everybody except rename does "lock parent, lookup,
 *	   lock child" and rename is under ->s_vfs_rename_mutex.
 *	   HOWEVER, it relies on the assumption that any object with ->lookup()
 *	   has no more than 1 dentry.  If "hybrid" objects will ever appear,
 *	   we'd better make sure that there's no link(2) for them.
 *	d) conversion from fhandle to dentry may come in the wrong moment - when
 *	   we are removing the target. Solution: we will have to grab ->i_mutex
 *	   in the fhandle_to_dentry code. [FIXME - current nfsfh.c relies on
 *	   ->i_mutex on parents, which works but leads to some truly excessive
 *	   locking].
 */
static int vfs_rename_dir(struct inode *old_dir, struct dentry *old_dentry,
			  struct inode *new_dir, struct dentry *new_dentry)
{
	int error = 0;
	struct inode *target = new_dentry->d_inode;

	/*
	 * If we are going to change the parent - check write permissions,
	 * we'll need to flip '..'.
	 */
	if (new_dir != old_dir) {
#ifdef CONFIG_FS_SYNO_ACL
		if (!IS_SYNOACL(old_dentry)) {
			error = inode_permission(old_dentry->d_inode, MAY_WRITE);
		}
#else
		error = inode_permission(old_dentry->d_inode, MAY_WRITE);
#endif
		if (error)
			return error;
	}

	error = security_inode_rename(old_dir, old_dentry, new_dir, new_dentry);
	if (error)
		return error;

	dget(new_dentry);
	if (target)
		mutex_lock(&target->i_mutex);

	error = -EBUSY;
	if (d_mountpoint(old_dentry) || d_mountpoint(new_dentry))
		goto out;

	if (target)
		shrink_dcache_parent(new_dentry);
	error = old_dir->i_op->rename(old_dir, old_dentry, new_dir, new_dentry);
	if (error)
		goto out;

	if (target) {
		target->i_flags |= S_DEAD;
		dont_mount(new_dentry);
	}
out:
	if (target)
		mutex_unlock(&target->i_mutex);
	dput(new_dentry);
	if (!error)
		if (!(old_dir->i_sb->s_type->fs_flags & FS_RENAME_DOES_D_MOVE))
			d_move(old_dentry,new_dentry);
	return error;
}

static int vfs_rename_other(struct inode *old_dir, struct dentry *old_dentry,
			    struct inode *new_dir, struct dentry *new_dentry)
{
	struct inode *target = new_dentry->d_inode;
	int error;

	error = security_inode_rename(old_dir, old_dentry, new_dir, new_dentry);
	if (error)
		return error;

	dget(new_dentry);
	if (target)
		mutex_lock(&target->i_mutex);

	error = -EBUSY;
	if (d_mountpoint(old_dentry)||d_mountpoint(new_dentry))
		goto out;

	error = old_dir->i_op->rename(old_dir, old_dentry, new_dir, new_dentry);
	if (error)
		goto out;

	if (target)
		dont_mount(new_dentry);
	if (!(old_dir->i_sb->s_type->fs_flags & FS_RENAME_DOES_D_MOVE))
		d_move(old_dentry, new_dentry);
out:
	if (target)
		mutex_unlock(&target->i_mutex);
	dput(new_dentry);
	return error;
}

int vfs_rename(struct inode *old_dir, struct dentry *old_dentry,
	       struct inode *new_dir, struct dentry *new_dentry)
{
	int error;
	int is_dir = S_ISDIR(old_dentry->d_inode->i_mode);
	const unsigned char *old_name;

#ifdef CONFIG_SYNO_NOTIFY
	char *tmp_old_full = NULL;
	char *tmp_new_full = NULL;
	char *tmp_old_buff = NULL;
	char *tmp_new_buff = NULL;
#endif
	if (old_dentry->d_inode == new_dentry->d_inode)
 		return 0;
 
	error = may_delete(old_dir, old_dentry, is_dir);
	if (error)
		return error;

	if (!new_dentry->d_inode){
#ifdef CONFIG_FS_SYNO_ACL
		error = may_create(new_dir, new_dentry, old_dentry->d_inode->i_mode);
#else
		error = may_create(new_dir, new_dentry);
#endif
	} else
		error = may_delete(new_dir, new_dentry, is_dir);
	if (error)
		return error;

	if (!old_dir->i_op->rename)
		return -EPERM;

#ifdef CONFIG_SYNO_NOTIFY
	tmp_old_buff = kmalloc(PATH_MAX, GFP_NOFS);
	tmp_new_buff = kmalloc(PATH_MAX, GFP_NOFS);
	tmp_old_full = dentry_path_raw(old_dentry, tmp_old_buff, PATH_MAX-1);
	tmp_new_full = dentry_path_raw(new_dentry, tmp_new_buff, PATH_MAX-1);
#endif
	old_name = fsnotify_oldname_init(old_dentry->d_name.name);

	if (is_dir)
		error = vfs_rename_dir(old_dir,old_dentry,new_dir,new_dentry);
	else
		error = vfs_rename_other(old_dir,old_dentry,new_dir,new_dentry);
	if (!error)
#ifdef CONFIG_SYNO_NOTIFY
		fsnotify_move(old_dir, new_dir, old_name, is_dir,
			      new_dentry->d_inode, old_dentry, tmp_old_full, tmp_new_full);
#else
		fsnotify_move(old_dir, new_dir, old_name, is_dir,
			      new_dentry->d_inode, old_dentry);
#endif
	fsnotify_oldname_free(old_name);

#ifdef CONFIG_SYNO_NOTIFY
	kfree(tmp_old_buff);
	kfree(tmp_new_buff);
#endif
	return error;
}

SYSCALL_DEFINE4(renameat, int, olddfd, const char __user *, oldname,
		int, newdfd, const char __user *, newname)
{
	struct dentry *old_dir, *new_dir;
	struct dentry *old_dentry, *new_dentry;
	struct dentry *trap;
	struct nameidata oldnd, newnd;
	char *from;
	char *to;
	int error;

	error = user_path_parent(olddfd, oldname, &oldnd, &from);
	if (error)
		goto exit;

	error = user_path_parent(newdfd, newname, &newnd, &to);
	if (error)
		goto exit1;

	error = -EXDEV;
	if (oldnd.path.mnt != newnd.path.mnt)
		goto exit2;

	old_dir = oldnd.path.dentry;
	error = -EBUSY;
	if (oldnd.last_type != LAST_NORM)
		goto exit2;

	new_dir = newnd.path.dentry;
	if (newnd.last_type != LAST_NORM)
		goto exit2;

	oldnd.flags &= ~LOOKUP_PARENT;
	newnd.flags &= ~LOOKUP_PARENT;
	newnd.flags |= LOOKUP_RENAME_TARGET;

	trap = lock_rename(new_dir, old_dir);

	old_dentry = lookup_hash(&oldnd);
	error = PTR_ERR(old_dentry);
	if (IS_ERR(old_dentry))
		goto exit3;
	/* source must exist */
	error = -ENOENT;
	if (!old_dentry->d_inode)
		goto exit4;
	/* unless the source is a directory trailing slashes give -ENOTDIR */
	if (!S_ISDIR(old_dentry->d_inode->i_mode)) {
		error = -ENOTDIR;
		if (oldnd.last.name[oldnd.last.len])
			goto exit4;
		if (newnd.last.name[newnd.last.len])
			goto exit4;
	}
	/* source should not be ancestor of target */
	error = -EINVAL;
	if (old_dentry == trap)
		goto exit4;
#ifdef MY_ABC_HERE
	if (new_dir->d_inode == old_dentry->d_inode) {
		/* only possible to happen in caseless filesystem */
		/* reject move myself into subdir of myself */
		error = -ENOENT;
		goto exit4;
	}
#endif
	new_dentry = lookup_hash(&newnd);
	error = PTR_ERR(new_dentry);
	if (IS_ERR(new_dentry))
		goto exit4;
	/* target should not be an ancestor of source */
	error = -ENOTEMPTY;
	if (new_dentry == trap)
		goto exit5;

	error = mnt_want_write(oldnd.path.mnt);
	if (error)
		goto exit5;
	error = security_path_rename(&oldnd.path, old_dentry,
				     &newnd.path, new_dentry);
	if (error)
		goto exit6;
	error = vfs_rename(old_dir->d_inode, old_dentry,
				   new_dir->d_inode, new_dentry);
exit6:
	mnt_drop_write(oldnd.path.mnt);
exit5:
	dput(new_dentry);
exit4:
	dput(old_dentry);
exit3:
	unlock_rename(new_dir, old_dir);
exit2:
	path_put(&newnd.path);
	putname(to);
exit1:
	path_put(&oldnd.path);
	putname(from);
exit:
	return error;
}

SYSCALL_DEFINE2(rename, const char __user *, oldname, const char __user *, newname)
{
	return sys_renameat(AT_FDCWD, oldname, AT_FDCWD, newname);
}

int vfs_readlink(struct dentry *dentry, char __user *buffer, int buflen, const char *link)
{
	int len;

	len = PTR_ERR(link);
	if (IS_ERR(link))
		goto out;

	len = strlen(link);
	if (len > (unsigned) buflen)
		len = buflen;
	if (copy_to_user(buffer, link, len))
		len = -EFAULT;
out:
	return len;
}

/*
 * A helper for ->readlink().  This should be used *ONLY* for symlinks that
 * have ->follow_link() touching nd only in nd_set_link().  Using (or not
 * using) it for any given inode is up to filesystem.
 */
int generic_readlink(struct dentry *dentry, char __user *buffer, int buflen)
{
	struct nameidata nd;
	void *cookie;
	int res;

	nd.depth = 0;
	cookie = dentry->d_inode->i_op->follow_link(dentry, &nd);
	if (IS_ERR(cookie))
		return PTR_ERR(cookie);

	res = vfs_readlink(dentry, buffer, buflen, nd_get_link(&nd));
	if (dentry->d_inode->i_op->put_link)
		dentry->d_inode->i_op->put_link(dentry, &nd, cookie);
	return res;
}

int vfs_follow_link(struct nameidata *nd, const char *link)
{
	return __vfs_follow_link(nd, link);
}

/* get the link contents into pagecache */
static char *page_getlink(struct dentry * dentry, struct page **ppage)
{
	char *kaddr;
	struct page *page;
	struct address_space *mapping = dentry->d_inode->i_mapping;
	page = read_mapping_page(mapping, 0, NULL);
	if (IS_ERR(page))
		return (char*)page;
	*ppage = page;
	kaddr = kmap(page);
	nd_terminate_link(kaddr, dentry->d_inode->i_size, PAGE_SIZE - 1);
	return kaddr;
}

int page_readlink(struct dentry *dentry, char __user *buffer, int buflen)
{
	struct page *page = NULL;
	char *s = page_getlink(dentry, &page);
	int res = vfs_readlink(dentry,buffer,buflen,s);
	if (page) {
		kunmap(page);
		page_cache_release(page);
	}
	return res;
}

void *page_follow_link_light(struct dentry *dentry, struct nameidata *nd)
{
	struct page *page = NULL;
	nd_set_link(nd, page_getlink(dentry, &page));
	return page;
}

void page_put_link(struct dentry *dentry, struct nameidata *nd, void *cookie)
{
	struct page *page = cookie;

	if (page) {
		kunmap(page);
		page_cache_release(page);
	}
}

/*
 * The nofs argument instructs pagecache_write_begin to pass AOP_FLAG_NOFS
 */
int __page_symlink(struct inode *inode, const char *symname, int len, int nofs)
{
	struct address_space *mapping = inode->i_mapping;
	struct page *page;
	void *fsdata;
	int err;
	char *kaddr;
	unsigned int flags = AOP_FLAG_UNINTERRUPTIBLE;
	if (nofs)
		flags |= AOP_FLAG_NOFS;

retry:
	err = pagecache_write_begin(NULL, mapping, 0, len-1,
				flags, &page, &fsdata);
	if (err)
		goto fail;

	kaddr = kmap_atomic(page, KM_USER0);
	memcpy(kaddr, symname, len-1);
	kunmap_atomic(kaddr, KM_USER0);

	err = pagecache_write_end(NULL, mapping, 0, len-1, len-1,
							page, fsdata);
	if (err < 0)
		goto fail;
	if (err < len-1)
		goto retry;

	mark_inode_dirty(inode);
	return 0;
fail:
	return err;
}

int page_symlink(struct inode *inode, const char *symname, int len)
{
	return __page_symlink(inode, symname, len,
			!(mapping_gfp_mask(inode->i_mapping) & __GFP_FS));
}

const struct inode_operations page_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= page_follow_link_light,
	.put_link	= page_put_link,
};

EXPORT_SYMBOL(user_path_at);
EXPORT_SYMBOL(follow_down_one);
EXPORT_SYMBOL(follow_down);
EXPORT_SYMBOL(follow_up);
#ifdef CONFIG_SYNO_NOTIFY
EXPORT_SYMBOL(syno_fetch_mountpoint_fullpath);
#endif
EXPORT_SYMBOL(get_write_access); /* binfmt_aout */
EXPORT_SYMBOL(getname);
EXPORT_SYMBOL(lock_rename);
EXPORT_SYMBOL(lookup_one_len);
#if defined(CONFIG_AUFS_FS ) || defined(MY_ABC_HERE)
EXPORT_SYMBOL(lookup_hash);
#endif
#ifdef MY_ABC_HERE
EXPORT_SYMBOL(kern_path_parent);
#endif
EXPORT_SYMBOL(page_follow_link_light);
EXPORT_SYMBOL(page_put_link);
EXPORT_SYMBOL(page_readlink);
EXPORT_SYMBOL(__page_symlink);
EXPORT_SYMBOL(page_symlink);
EXPORT_SYMBOL(page_symlink_inode_operations);
EXPORT_SYMBOL(kern_path);
EXPORT_SYMBOL(vfs_path_lookup);
EXPORT_SYMBOL(inode_permission);
EXPORT_SYMBOL(unlock_rename);
EXPORT_SYMBOL(vfs_create);
EXPORT_SYMBOL(vfs_follow_link);
EXPORT_SYMBOL(vfs_link);
EXPORT_SYMBOL(vfs_mkdir);
EXPORT_SYMBOL(vfs_mknod);
EXPORT_SYMBOL(generic_permission);
EXPORT_SYMBOL(vfs_readlink);
EXPORT_SYMBOL(vfs_rename);
EXPORT_SYMBOL(vfs_rmdir);
EXPORT_SYMBOL(vfs_symlink);
EXPORT_SYMBOL(vfs_unlink);
EXPORT_SYMBOL(dentry_unhash);
EXPORT_SYMBOL(generic_readlink);
