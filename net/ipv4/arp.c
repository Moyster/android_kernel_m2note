/* linux/net/ipv4/arp.c
 *
 * Copyright (C) 1994 by Florian  La Roche
 *
 * This module implements the Address Resolution Protocol ARP (RFC 826),
 * which is used to convert IP addresses (or in the future maybe other
 * high-level addresses) into a low-level hardware address (like an Ethernet
 * address).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * Fixes:
 *		Alan Cox	:	Removed the Ethernet assumptions in
 *					Florian's code
 *		Alan Cox	:	Fixed some small errors in the ARP
 *					logic
 *		Alan Cox	:	Allow >4K in /proc
 *		Alan Cox	:	Make ARP add its own protocol entry
 *		Ross Martin     :       Rewrote arp_rcv() and arp_get_info()
 *		Stephen Henson	:	Add AX25 support to arp_get_info()
 *		Alan Cox	:	Drop data when a device is downed.
 *		Alan Cox	:	Use init_timer().
 *		Alan Cox	:	Double lock fixes.
 *		Martin Seine	:	Move the arphdr structure
 *					to if_arp.h for compatibility.
 *					with BSD based programs.
 *		Andrew Tridgell :       Added ARP netmask code and
 *					re-arranged proxy handling.
 *		Alan Cox	:	Changed to use notifiers.
 *		Niibe Yutaka	:	Reply for this device or proxies only.
 *		Alan Cox	:	Don't proxy across hardware types!
 *		Jonathan Naylor :	Added support for NET/ROM.
 *		Mike Shaver     :       RFC1122 checks.
 *		Jonathan Naylor :	Only lookup the hardware address for
 *					the correct hardware type.
 *		Germano Caronni	:	Assorted subtle races.
 *		Craig Schlenter :	Don't modify permanent entry
 *					during arp_rcv.
 *		Russ Nelson	:	Tidied up a few bits.
 *		Alexey Kuznetsov:	Major changes to caching and behaviour,
 *					eg intelligent arp probing and
 *					generation
 *					of host down events.
 *		Alan Cox	:	Missing unlock in device events.
 *		Eckes		:	ARP ioctl control errors.
 *		Alexey Kuznetsov:	Arp free fix.
 *		Manuel Rodriguez:	Gratuitous ARP.
 *              Jonathan Layes  :       Added arpd support through kerneld
 *                                      message queue (960314)
 *		Mike Shaver	:	/proc/sys/net/ipv4/arp_* support
 *		Mike McLagan    :	Routing by source
 *		Stuart Cheshire	:	Metricom and grat arp fixes
 *					*** FOR 2.1 clean this up ***
 *		Lawrence V. Stefani: (08/12/96) Added FDDI support.
 *		Alan Cox	:	Took the AP1000 nasty FDDI hack and
 *					folded into the mainstream FDDI code.
 *					Ack spit, Linus how did you allow that
 *					one in...
 *		Jes Sorensen	:	Make FDDI work again in 2.1.x and
 *					clean up the APFDDI & gen. FDDI bits.
 *		Alexey Kuznetsov:	new arp state machine;
 *					now it is in net/core/neighbour.c.
 *		Krzysztof Halasa:	Added Frame Relay ARP support.
 *		Arnaldo C. Melo :	convert /proc/net/arp to seq_file
 *		Shmulik Hen:		Split arp_send to arp_create and
 *					arp_xmit so intermediate drivers like
 *					bonding can change the skb before
 *					sending (e.g. insert 8021q tag).
 *		Harald Welte	:	convert to make use of jenkins hash
 *		Jesper D. Brouer:       Proxy ARP PVLAN RFC 3069 support.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/capability.h>
#include <linux/socket.h>
#include <linux/sockios.h>
#include <linux/errno.h>
#include <linux/in.h>
#include <linux/mm.h>
#include <linux/inet.h>
#include <linux/inetdevice.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/fddidevice.h>
#include <linux/skbuff.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/net.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#ifdef CONFIG_SYSCTL
#include <linux/sysctl.h>
#endif

#include <net/net_namespace.h>
#include <net/ip.h>
#include <net/icmp.h>
#include <net/route.h>
#include <net/protocol.h>
#include <net/tcp.h>
#include <net/sock.h>
#include <net/arp.h>
#include <net/ax25.h>
#include <net/netrom.h>

#include <linux/uaccess.h>

#include <linux/netfilter_arp.h>

/* arp_project */
#include <net/arp_project.h>

bool arp_project_enable = true;
bool print_arp_info = false;
static bool ignore_gw_update_by_request = true;
static bool ignore_gw_update_by_reply = true;
EXPORT_SYMBOL(arp_project_enable);
EXPORT_SYMBOL(print_arp_info);

extern __be32 ip_fib_get_gw(struct net_device *dev);

/*
 *	Interface to generic neighbour cache.
 */
static u32 arp_hash(const void *pkey, const struct net_device *dev, __u32 *hash_rnd);
static int arp_constructor(struct neighbour *neigh);
static void arp_solicit(struct neighbour *neigh, struct sk_buff *skb);
static void arp_error_report(struct neighbour *neigh, struct sk_buff *skb);
static void parp_redo(struct sk_buff *skb);

static const struct neigh_ops arp_generic_ops = {
	.family =		AF_INET,
	.solicit =		arp_solicit,
	.error_report =		arp_error_report,
	.output =		neigh_resolve_output,
	.connected_output =	neigh_connected_output,
};

static const struct neigh_ops arp_hh_ops = {
	.family =		AF_INET,
	.solicit =		arp_solicit,
	.error_report =		arp_error_report,
	.output =		neigh_resolve_output,
	.connected_output =	neigh_resolve_output,
};

static const struct neigh_ops arp_direct_ops = {
	.family =		AF_INET,
	.output =		neigh_direct_output,
	.connected_output =	neigh_direct_output,
};

/* MTK_NET_CHANGES */
struct neigh_table arp_tbl = {
	.family		= AF_INET,
	.key_len	= 4,
	.hash		= arp_hash,
	.constructor	= arp_constructor,
	.proxy_redo	= parp_redo,
	.id		= "arp_cache",
	.parms		= {
		.tbl			= &arp_tbl,
		.base_reachable_time	= 30 * HZ,
		.retrans_time		= 1 * HZ,
		.gc_staletime		= 60 * HZ,
		.reachable_time		= 30 * HZ,
		.delay_probe_time	= 5 * HZ,
		.queue_len_bytes	= 64*1024,
		.ucast_probes		= 3,
		.mcast_probes		= 3,
		.anycast_delay		= 1 * HZ,
		.proxy_delay		= (8 * HZ) / 10,
		.proxy_qlen		= 64,
		.locktime		= 1 * HZ,
	},
	.gc_interval	= 30 * HZ,
	.gc_thresh1	= 512,
	.gc_thresh2	= 1024,
	.gc_thresh3	= 2048,
};
EXPORT_SYMBOL(arp_tbl);

int arp_mc_map(__be32 addr, u8 *haddr, struct net_device *dev, int dir)
{
	switch (dev->type) {
	case ARPHRD_ETHER:
	case ARPHRD_FDDI:
	case ARPHRD_IEEE802:
		ip_eth_mc_map(addr, haddr);
		return 0;
	case ARPHRD_INFINIBAND:
		ip_ib_mc_map(addr, dev->broadcast, haddr);
		return 0;
	case ARPHRD_IPGRE:
		ip_ipgre_mc_map(addr, dev->broadcast, haddr);
		return 0;
	default:
		if (dir) {
			memcpy(haddr, dev->broadcast, dev->addr_len);
			return 0;
		}
	}
	return -EINVAL;
}


static u32 arp_hash(const void *pkey,
		    const struct net_device *dev,
		    __u32 *hash_rnd)
{
	return arp_hashfn(*(u32 *)pkey, dev, *hash_rnd);
}

static int arp_constructor(struct neighbour *neigh)
{
	__be32 addr = *(__be32 *)neigh->primary_key;
	struct net_device *dev = neigh->dev;
	struct in_device *in_dev;
	struct neigh_parms *parms;

	rcu_read_lock();
	in_dev = __in_dev_get_rcu(dev);
	if (in_dev == NULL) {
		rcu_read_unlock();
		return -EINVAL;
	}

	neigh->type = inet_addr_type(dev_net(dev), addr);

	parms = in_dev->arp_parms;
	__neigh_parms_put(neigh->parms);
	neigh->parms = neigh_parms_clone(parms);
	rcu_read_unlock();

	if (!dev->header_ops) {
		neigh->nud_state = NUD_NOARP;
		neigh->ops = &arp_direct_ops;
		neigh->output = neigh_direct_output;
	} else {
		/* Good devices (checked by reading texts, but only Ethernet is
		   tested)

		   ARPHRD_ETHER: (ethernet, apfddi)
		   ARPHRD_FDDI: (fddi)
		   ARPHRD_IEEE802: (tr)
		   ARPHRD_METRICOM: (strip)
		   ARPHRD_ARCNET:
		   etc. etc. etc.

		   ARPHRD_IPDDP will also work, if author repairs it.
		   I did not it, because this driver does not work even
		   in old paradigm.
		 */

		if (neigh->type == RTN_MULTICAST) {
			neigh->nud_state = NUD_NOARP;
			arp_mc_map(addr, neigh->ha, dev, 1);
		} else if (dev->flags & (IFF_NOARP | IFF_LOOPBACK)) {
			neigh->nud_state = NUD_NOARP;
			memcpy(neigh->ha, dev->dev_addr, dev->addr_len);
		} else if (neigh->type == RTN_BROADCAST ||
			   (dev->flags & IFF_POINTOPOINT)) {
			neigh->nud_state = NUD_NOARP;
			memcpy(neigh->ha, dev->broadcast, dev->addr_len);
		}

		if (dev->header_ops->cache)
			neigh->ops = &arp_hh_ops;
		else
			neigh->ops = &arp_generic_ops;

		if (neigh->nud_state & NUD_VALID)
			neigh->output = neigh->ops->connected_output;
		else
			neigh->output = neigh->ops->output;
	}
	return 0;
}

static void arp_error_report(struct neighbour *neigh, struct sk_buff *skb)
{
	dst_link_failure(skb);
	kfree_skb(skb);
}

/*
 * arp_project
 *
 * Print ARP packet informations
 *
 * @dev - net device
 * @arp - arp header
 * @count - 0: Recevied ARP, 1: Sending ARP
 */
void arp_print_info(struct net_device *dev, struct arphdr *arp, int count)
{
	unsigned char *arp_ptr;
	int i;

	printk(ARP_PROJECT"%s - =============== ARP Info ===============\n",
		__func__);

	/* net_device info */
	if (count)
		printk(ARP_PROJECT"%s - Sending dev_addr: ", __func__);
	else
		printk(ARP_PROJECT"%s - Received dev_addr: ", __func__);
	for (i = 0; i < dev->addr_len - 1; i++)
		printk("%02x:", dev->dev_addr[i]);
	printk("%02x\n", dev->dev_addr[i]);

	/* operation info */
	if (arp->ar_op == htons(ARPOP_REQUEST))
		printk(ARP_PROJECT"%s - Operation: Request(1)\n", __func__);
	else if (arp->ar_op == htons(ARPOP_REPLY))
		printk(ARP_PROJECT"%s - Operation: Reply(2)\n", __func__);

	/* Get arp_ptr infos */
	arp_ptr = (unsigned char *)(arp + 1);

	/* Sender Hardware Address info */
	printk(ARP_PROJECT"%s - Sender HW: ", __func__);
	for (i = 0; i < dev->addr_len - 1; i++)
		printk("%02x:", arp_ptr[i]);
	printk("%02x\n", arp_ptr[i]);

	/* Move pointer */
	arp_ptr += dev->addr_len;

	printk(ARP_PROJECT"%s - Sender IP: ", __func__);
	for (i = 0; i < 3; i++)
		printk("%d.", arp_ptr[i]);
	printk("%d\n", arp_ptr[i]);

	/* Move pointer */
	arp_ptr += 4;

	/* Target Hardware Address info */
	printk(ARP_PROJECT"%s - Target HW: ", __func__);
	for (i = 0; i < dev->addr_len - 1; i++)
		printk("%02x:", arp_ptr[i]);
	printk("%02x\n", arp_ptr[i]);

	/* Move pointer */
	arp_ptr += dev->addr_len;

	/* Target IP Address info */
	printk(ARP_PROJECT"%s - Target IP: ", __func__);
	for (i = 0; i < 3; i++)
		printk("%d.", arp_ptr[i]);
	printk("%d\n", arp_ptr[i]);
}
EXPORT_SYMBOL(arp_print_info);

/*
 * arp_project
 *
 *  Print information of sending ARP packet and
 * check its data from user buffer.
 *  This function is called by "sendto" sys call.
 *  See "net/socket.c".
 */
int arp_print_and_check_send_user(const struct socket *sock,
				  void __user *buff, size_t len,
				  const struct sockaddr_ll *saddr)
{
	struct sock *sk = sock->sk;
	struct arphdr *arp;
	struct net_device *dev;
	struct in_device *in_dev;
	struct in_ifaddr **ifap = NULL;
	struct in_ifaddr *ifa = NULL;
	void *kdata = NULL;
	unsigned char *arp_ptr;
	__be32 sip;
	bool ip_is_mine = false;
	int err;

	dev = dev_get_by_index(sock_net(sk), saddr->sll_ifindex);
	if (!dev) {
		printk(ARP_PROJECT"%s: Device not found.\n", __func__);
		err = -ENODEV;
		goto out;
	}

	if (dev->flags & IFF_NOARP) {
		printk(ARP_PROJECT"%s: Device not support ARP.\n", __func__);
		err = -EPERM;
		goto put_dev;
	}

	if (len < arp_hdr_len(dev)) {
		printk(ARP_PROJECT"%s: Length is too short.\n", __func__);
		err = -EINVAL;
		goto put_dev;
	}

	kdata = kzalloc(len, GFP_KERNEL);
	if (!kdata) {
		printk(ARP_PROJECT"%s: Failed to allocate kernel buffer.\n", __func__);
		err = -ENOMEM;
		goto put_dev;
	}

	if (copy_from_user(kdata, buff, len)) {
		printk(ARP_PROJECT"%s: Failed to copy user data.\n", __func__);
		err = -EFAULT;
		goto put_dev;
	}

	arp = (struct arphdr *)kdata;

	/* Print arp_ptr infos */
	if (print_arp_info)
		arp_print_info(dev, arp, 1);

	arp_ptr = (unsigned char *)(arp + 1); // Next to the ARP Header (arp_hdr)

	/* Check hardware address */
	if (memcmp(dev->dev_addr, arp_ptr, dev->addr_len) != 0) {
		err = -EPERM;
		goto not_mine;
	}

	/* Get source IP address and destination IP address */
	arp_ptr += (dev->addr_len); // Skip source hardware address
	memcpy(&sip, arp_ptr, 4); // Get source IP address
	arp_ptr += (4 + dev->addr_len); // Skip source IP and target hardware address

	rcu_read_lock();
	in_dev = __in_dev_get_rcu(dev);
	if (in_dev == NULL) {
		printk(ARP_PROJECT"%s: Failed to get in_device.\n", __func__);
		rcu_read_unlock();
		err = -EINVAL;
		goto kfree;
	}

	/* Check IP address */
	for (ifap = &in_dev->ifa_list; (ifa = *ifap) != NULL;
	     ifap = &ifa->ifa_next) {
		if (print_arp_info) {
			int i;
			unsigned char ip_tmp[4];

			memcpy(&ip_tmp, &ifa->ifa_local, 4);
			printk(ARP_PROJECT"%s: Device's ip addr: ",
				__func__);
			for (i = 0; i < 3; i++)
				printk("%d.", ip_tmp[i]);
			printk("%d\n", ip_tmp[i]);
		}

		if (memcmp(&sip, &ifa->ifa_local, 4) == 0) {
			ip_is_mine = true;
			break;
		}
	}
	rcu_read_unlock();

	if (!ip_is_mine) {
		err = -EPERM;
		goto not_mine;
	}

	err = 0;
	goto kfree;

not_mine:
	printk(ARP_PROJECT"%s: Source MAC or Source IP is not yours.\n",
			__func__);
kfree:
	kfree(kdata);
put_dev:
	dev_put(dev);
out:
	return err;
}
EXPORT_SYMBOL(arp_print_and_check_send_user);

/* Create and send an arp packet. */
static void arp_send_dst(int type, int ptype, __be32 dest_ip,
			 struct net_device *dev, __be32 src_ip,
			 const unsigned char *dest_hw,
			 const unsigned char *src_hw,
			 const unsigned char *target_hw, struct sk_buff *oskb)
{
	struct sk_buff *skb;

	/* arp on this interface. */
	if (dev->flags & IFF_NOARP)
		return;
	#ifdef CONFIG_MTK_NET_LOGGING  	
    printk(KERN_INFO "[mtk_net][arp]arp_send type = %d, dev = %s\n", type, dev->name);
    #endif
	skb = arp_create(type, ptype, dest_ip, dev, src_ip,
			 dest_hw, src_hw, target_hw);
	if (!skb)
		return;

	/* arp_project - Print arp_ptr infos */
	if (arp_project_enable && print_arp_info) {
		struct arphdr *arp;

		arp = arp_hdr(skb);
		arp_print_info(dev, arp, 1);
	}

	if (oskb)
		skb_dst_copy(skb, oskb);

	arp_xmit(skb);
}

void arp_send(int type, int ptype, __be32 dest_ip,
	      struct net_device *dev, __be32 src_ip,
	      const unsigned char *dest_hw, const unsigned char *src_hw,
	      const unsigned char *target_hw)
{
	arp_send_dst(type, ptype, dest_ip, dev, src_ip, dest_hw, src_hw,
		     target_hw, NULL);
}
EXPORT_SYMBOL(arp_send);

static void arp_solicit(struct neighbour *neigh, struct sk_buff *skb)
{
	__be32 saddr = 0;
	u8 dst_ha[MAX_ADDR_LEN], *dst_hw = NULL;
	struct net_device *dev = neigh->dev;
	__be32 target = *(__be32 *)neigh->primary_key;
	int probes = atomic_read(&neigh->probes);
	struct in_device *in_dev;

	rcu_read_lock();
	in_dev = __in_dev_get_rcu(dev);
	if (!in_dev) {
		rcu_read_unlock();
		return;
	}
	switch (IN_DEV_ARP_ANNOUNCE(in_dev)) {
	default:
	case 0:		/* By default announce any local IP */
		if (skb && inet_addr_type(dev_net(dev),
					  ip_hdr(skb)->saddr) == RTN_LOCAL)
			saddr = ip_hdr(skb)->saddr;
		break;
	case 1:		/* Restrict announcements of saddr in same subnet */
		if (!skb)
			break;
		saddr = ip_hdr(skb)->saddr;
		if (inet_addr_type(dev_net(dev), saddr) == RTN_LOCAL) {
			/* saddr should be known to target */
			if (inet_addr_onlink(in_dev, target, saddr))
				break;
		}
		saddr = 0;
		break;
	case 2:		/* Avoid secondary IPs, get a primary/preferred one */
		break;
	}
	rcu_read_unlock();

	if (!saddr)
		saddr = inet_select_addr(dev, target, RT_SCOPE_LINK);

	probes -= neigh->parms->ucast_probes;
	if (probes < 0) {
		if (!(neigh->nud_state & NUD_VALID))
			pr_debug("trying to ucast probe in NUD_INVALID\n");
		neigh_ha_snapshot(dst_ha, neigh, dev);
		dst_hw = dst_ha;
	} else {
		probes -= neigh->parms->app_probes;
		if (probes < 0) {
			neigh_app_ns(neigh);
			return;
		}
	}

	arp_send_dst(ARPOP_REQUEST, ETH_P_ARP, target, dev, saddr,
		     dst_hw, dev->dev_addr, NULL,
		     dev->priv_flags & IFF_XMIT_DST_RELEASE ? NULL : skb);
}

static int arp_ignore(struct in_device *in_dev, __be32 sip, __be32 tip)
{
	int scope;

	switch (IN_DEV_ARP_IGNORE(in_dev)) {
	case 0:	/* Reply, the tip is already validated */
		return 0;
	case 1:	/* Reply only if tip is configured on the incoming interface */
		sip = 0;
		scope = RT_SCOPE_HOST;
		break;
	case 2:	/*
		 * Reply only if tip is configured on the incoming interface
		 * and is in same subnet as sip
		 */
		scope = RT_SCOPE_HOST;
		break;
	case 3:	/* Do not reply for scope host addresses */
		sip = 0;
		scope = RT_SCOPE_LINK;
		break;
	case 4:	/* Reserved */
	case 5:
	case 6:
	case 7:
		return 0;
	case 8:	/* Do not reply */
		return 1;
	default:
		return 0;
	}
	return !inet_confirm_addr(in_dev, sip, tip, scope);
}

static int arp_filter(__be32 sip, __be32 tip, struct net_device *dev)
{
	struct rtable *rt;
	int flag = 0;
	/*unsigned long now; */
	struct net *net = dev_net(dev);

	rt = ip_route_output(net, sip, tip, 0, 0);
	if (IS_ERR(rt))
		return 1;
	if (rt->dst.dev != dev) {
		NET_INC_STATS_BH(net, LINUX_MIB_ARPFILTER);
		flag = 1;
	}
	ip_rt_put(rt);
	return flag;
}

/* OBSOLETE FUNCTIONS */

/*
 *	Find an arp mapping in the cache. If not found, post a request.
 *
 *	It is very UGLY routine: it DOES NOT use skb->dst->neighbour,
 *	even if it exists. It is supposed that skb->dev was mangled
 *	by a virtual device (eql, shaper). Nobody but broken devices
 *	is allowed to use this function, it is scheduled to be removed. --ANK
 */

static int arp_set_predefined(int addr_hint, unsigned char *haddr,
			      __be32 paddr, struct net_device *dev)
{
	switch (addr_hint) {
	case RTN_LOCAL:
		pr_debug("arp called for own IP address\n");
		memcpy(haddr, dev->dev_addr, dev->addr_len);
		return 1;
	case RTN_MULTICAST:
		arp_mc_map(paddr, haddr, dev, 1);
		return 1;
	case RTN_BROADCAST:
		memcpy(haddr, dev->broadcast, dev->addr_len);
		return 1;
	}
	return 0;
}


int arp_find(unsigned char *haddr, struct sk_buff *skb)
{
	struct net_device *dev = skb->dev;
	__be32 paddr;
	struct neighbour *n;

	if (!skb_dst(skb)) {
		pr_debug("arp_find is called with dst==NULL\n");
		kfree_skb(skb);
		return 1;
	}

	paddr = rt_nexthop(skb_rtable(skb), ip_hdr(skb)->daddr);
	if (arp_set_predefined(inet_addr_type(dev_net(dev), paddr), haddr,
			       paddr, dev))
		return 0;

	n = __neigh_lookup(&arp_tbl, &paddr, dev, 1);

	if (n) {
		n->used = jiffies;
		if (n->nud_state & NUD_VALID || neigh_event_send(n, skb) == 0) {
			neigh_ha_snapshot(haddr, n, dev);
			neigh_release(n);
			return 0;
		}
		neigh_release(n);
	} else
		kfree_skb(skb);
	return 1;
}
EXPORT_SYMBOL(arp_find);

/* END OF OBSOLETE FUNCTIONS */

/*
 * Check if we can use proxy ARP for this path
 */
static inline int arp_fwd_proxy(struct in_device *in_dev,
				struct net_device *dev,	struct rtable *rt)
{
	struct in_device *out_dev;
	int imi, omi = -1;

	if (rt->dst.dev == dev)
		return 0;

	if (!IN_DEV_PROXY_ARP(in_dev))
		return 0;
	imi = IN_DEV_MEDIUM_ID(in_dev);
	if (imi == 0)
		return 1;
	if (imi == -1)
		return 0;

	/* place to check for proxy_arp for routes */

	out_dev = __in_dev_get_rcu(rt->dst.dev);
	if (out_dev)
		omi = IN_DEV_MEDIUM_ID(out_dev);

	return omi != imi && omi != -1;
}

/*
 * Check for RFC3069 proxy arp private VLAN (allow to send back to same dev)
 *
 * RFC3069 supports proxy arp replies back to the same interface.  This
 * is done to support (ethernet) switch features, like RFC 3069, where
 * the individual ports are not allowed to communicate with each
 * other, BUT they are allowed to talk to the upstream router.  As
 * described in RFC 3069, it is possible to allow these hosts to
 * communicate through the upstream router, by proxy_arp'ing.
 *
 * RFC 3069: "VLAN Aggregation for Efficient IP Address Allocation"
 *
 *  This technology is known by different names:
 *    In RFC 3069 it is called VLAN Aggregation.
 *    Cisco and Allied Telesyn call it Private VLAN.
 *    Hewlett-Packard call it Source-Port filtering or port-isolation.
 *    Ericsson call it MAC-Forced Forwarding (RFC Draft).
 *
 */
static inline int arp_fwd_pvlan(struct in_device *in_dev,
				struct net_device *dev,	struct rtable *rt,
				__be32 sip, __be32 tip)
{
	/* Private VLAN is only concerned about the same ethernet segment */
	if (rt->dst.dev != dev)
		return 0;

	/* Don't reply on self probes (often done by windowz boxes)*/
	if (sip == tip)
		return 0;

	if (IN_DEV_PROXY_ARP_PVLAN(in_dev))
		return 1;
	else
		return 0;
}

/*
 *	Interface to link layer: send routine and receive handler.
 */

/*
 *	Create an arp packet. If (dest_hw == NULL), we create a broadcast
 *	message.
 */

/*

 arp_project

 == ARP Header Structure ==

 0                    7                         15                               31
 |                    |                         |                                |
 |-------------------------------------------------------------------------------|<- arp <- skb->data
 |    <Hardware Type>   (Ethernet = 1)          | <Protocol Type> (IPv4: 0x0800) | |
 |    arp->ar_hrd = htons(dev->type);           | arp->ar_pro = htons(ETH_P_IP); | |
 |-------------------------------------------------------------------------------| |
 | <HW Address Length> | <Protocol Addr Length> | <Operation Code>               | |<- sizeof(struct arp_hdr)
 | (MAC: 6byte)        | (IPv4 Length: 4byte)   | int type; ---- ARPOP_REQUEST 1 | |
 | arp->hln =          | arp->ar_pln = 4;       |            |-- ARPOP_REPLY   2 | |
 |    dev->addr_len;   |                        | arp->ar_op = htons(type);      | |
 |-------------------------------------------------------------------------------|<- arp + 1 <- arp_ptr
 | <Sender Hardware Address>  (6byte MAC Address)                                | |
 | memcpy(arp_ptr, src_hw, dev->addr_len);                                       | |
 | arp_ptr += dev->addr_len;                                                     | |
 |                                              |--------------------------------| |
 |                                              |                                | |
 |                                              |                                | |
 |                                              |                                | |
 |----------------------------------------------|--------------------------------| |
 | <Sender Protocol Address> (4byte IP Address) |                                | |
 | memcpy(arp_ptr, &src_ip, 4);                 |                                | |<- (dev->addr_len +
 | arp_ptr += 4;                                |                                | |     sizeof(u32)) * 2
 |----------------------------------------------|                                | |
 | <Target Hardware Address>  (6byte MAC Address)                                | |
 | if(target_hw != NULL) memcpy(arp_ptr, target_hw, dev->addr_len);              | |
 | else memcpy(arp_ptr, 0, dev->addr_len);           arp_ptr += dev->addr_len;   | |
 |-------------------------------------------------------------------------------| |
 | <Target Protocol Address> (4byte IP Address)                                  | |
 | memcpy(arp_ptr, &dest_ip, 4);                                                 | |
 |                                                                               | |
 |-------------------------------------------------------------------------------|<- skb->tail


 == skb Structure ==

 skb->head -->|----------------------------|
              |            head            |   decrease head: skb_push()
 skb->data -->|----------------------------|------
              |                            |   increase head: skb_pull()
              |            data            |
              |                            |
 skb->tail -->|----------------------------|------
              |            tail            |    increase data: skb_put()
 skb->end  -->|----------------------------|


 @ type       ---- ARPOP_REQUEST 1
               |-- ARPOP_REPLY   2
 @ ptype      If Ethernet - ETH_P_ARP
 @ dest_ip    Destination IP Address
 @ dev        Network Device
 @ dest_ip    Destination IP Address
 @ src_ip     Source IP Address
 @ dest_hw    Destination HW Address
 @ src_hw     Source HW Address
 @ target_hw  Target HW Address   (REPLY - same with dest_hw / REQUEST - NULL)
*/

struct sk_buff *arp_create(int type, int ptype, __be32 dest_ip,
			   struct net_device *dev, __be32 src_ip,
			   const unsigned char *dest_hw,
			   const unsigned char *src_hw,
			   const unsigned char *target_hw)
{
	struct sk_buff *skb;
	struct arphdr *arp;
	unsigned char *arp_ptr;
	int hlen = LL_RESERVED_SPACE(dev); // dev header + dev header room
	int tlen = dev->needed_tailroom;

	/*
	 *	Allocate a buffer
	 */

	skb = alloc_skb(arp_hdr_len(dev) + hlen + tlen, GFP_ATOMIC);
	if (skb == NULL)
		return NULL;

	skb_reserve(skb, hlen);	// reserve skb header (dev header + dev header room)
	skb_reset_network_header(skb);
	arp = (struct arphdr *) skb_put(skb, arp_hdr_len(dev)); // reserve data space
	/*

	arp_project

	 == arp_hdr_len(dev) ==

	<linux/if_arp.h>
	case ...IEEE_1394:
		...
	default: // (Ethernet is here)
		return (sizeof(struct arp_hdr) + (dev->addr_len + sizeof(u32)) * 2;
			// ARP Header + 2 device addresses + 2 IP addresses)
	*/
	skb->dev = dev;
	skb->protocol = htons(ETH_P_ARP);
	if (src_hw == NULL)
		src_hw = dev->dev_addr;
	if (dest_hw == NULL)	// If REQUEST we should create a broadcast.
		dest_hw = dev->broadcast;

	/*
	 *	Fill the device header for the ARP frame
	 */
	// If Ethernet, ptype = ETH_P_ARP
	if (dev_hard_header(skb, dev, ptype, dest_hw, src_hw, skb->len) < 0)
		goto out;

	/*
	 * Fill out the arp protocol part.
	 *
	 * The arp hardware type should match the device type, except for FDDI,
	 * which (according to RFC 1390) should always equal 1 (Ethernet).
	 */
	/*
	 *	Exceptions everywhere. AX.25 uses the AX.25 PID value not the
	 *	DIX code for the protocol. Make these device structure fields.
	 */
	switch (dev->type) {
	default:	// Ethernet is here
		arp->ar_hrd = htons(dev->type);
		arp->ar_pro = htons(ETH_P_IP);
		break;

#if IS_ENABLED(CONFIG_AX25)
	case ARPHRD_AX25:
		arp->ar_hrd = htons(ARPHRD_AX25);
		arp->ar_pro = htons(AX25_P_IP);
		break;

#if IS_ENABLED(CONFIG_NETROM)
	case ARPHRD_NETROM:
		arp->ar_hrd = htons(ARPHRD_NETROM);
		arp->ar_pro = htons(AX25_P_IP);
		break;
#endif
#endif

#if IS_ENABLED(CONFIG_FDDI)
	case ARPHRD_FDDI:
		arp->ar_hrd = htons(ARPHRD_ETHER);
		arp->ar_pro = htons(ETH_P_IP);
		break;
#endif
	}

	arp->ar_hln = dev->addr_len;
	arp->ar_pln = 4;
	arp->ar_op = htons(type);

	/*

	arp_project

	 skb_put reserved size of arphdr(ARP header) + 2 MACs + 2 IPs.
	 But remember the variable *arp 's type is arphdr.
	 So we can put arp data after (arp + 1) that end of ARP header.
	*/
	arp_ptr = (unsigned char *)(arp + 1);	// Next to the ARP Header (arp_hdr)

	memcpy(arp_ptr, src_hw, dev->addr_len);
	arp_ptr += dev->addr_len;
	memcpy(arp_ptr, &src_ip, 4);	// 32bit IP address -> 4byte
	arp_ptr += 4;

	switch (dev->type) {
#if IS_ENABLED(CONFIG_FIREWIRE_NET)
	case ARPHRD_IEEE1394:
		break;
#endif
	default:
		if (target_hw != NULL)
			memcpy(arp_ptr, target_hw, dev->addr_len);
		else
			memset(arp_ptr, 0, dev->addr_len);
		arp_ptr += dev->addr_len;
	}
	memcpy(arp_ptr, &dest_ip, 4);

	return skb;

out:
	kfree_skb(skb);
	return NULL;
}
EXPORT_SYMBOL(arp_create);

/*
 *	Send an arp packet.
 */
void arp_xmit(struct sk_buff *skb)
{
	/* Send it off, maybe filter it using firewalling first.  */
	NF_HOOK(NFPROTO_ARP, NF_ARP_OUT, skb, NULL, skb->dev, dev_queue_xmit);
}
EXPORT_SYMBOL(arp_xmit);

/*
 * arp_project
 *
 * Find default gateway and check attempt of gateway update.
 *
 * 0 - Default gateway not found or normal request.
 * 1 - Default gateway found and gateway update detected from other hardware address.
 */
static int arp_detect_gw_update(struct net_device *dev, __be32 sip,
			       unsigned char *sha)
{
	struct neighbour *n;
	__be32 gw = ip_fib_get_gw(dev);
	int found = 0;
	int sum = 0;
	int i;

	if (!gw)
		return found;

	if (print_arp_info) {
		unsigned char ip_tmp[4];

		memcpy(&ip_tmp, &gw, 4);
		printk(ARP_PROJECT"%s - Gateway IP: ", __func__);
		for (i = 0; i < 3; i++)
			printk("%d.", ip_tmp[i]);
		printk("%d\n", ip_tmp[i]);
	}

	if (sip != gw)
		return found;

	n = neigh_lookup(&arp_tbl, &sip, dev);
	if (n) {
		if (print_arp_info) {
			printk(ARP_PROJECT"%s - Gateway HW: ", __func__);
			for (i = 0; i < dev->addr_len - 1; i++)
				printk("%02x:", n->ha[i]);
			printk("%02x\n", n->ha[i]);
		}

		/* Zero check - Incomplete state */
		for (i = 0; i < dev->addr_len; i++)
			sum += n->ha[i];
		if(!sum) {
			neigh_release(n);
			return found;
		}

		if (memcmp(n->ha, sha, dev->addr_len)) {
			printk(ARP_PROJECT"%s: Gateway update attempt detected from ",
									__func__);
			for (i = 0; i < dev->addr_len - 1; i++)
				printk("%02x:", sha[i]);
			printk("%02x !\n", sha[i]);

			found = 1;
		}
		neigh_release(n);
	}

	return found;
}

/*
 *	Process an arp request.
 */
static int arp_process(struct sk_buff *skb)
{
	struct net_device *dev = skb->dev;
	struct in_device *in_dev = __in_dev_get_rcu(dev);
	struct arphdr *arp;
	unsigned char *arp_ptr;
	struct rtable *rt;
	unsigned char *sha; // Sender Hardware Address
	__be32 sip, tip;
	u16 dev_type = dev->type;
	int addr_type;
	struct neighbour *n;
	struct net *net = dev_net(dev);
	bool is_garp = false;

	/* arp_rcv below verifies the ARP header and verifies the device
	 * is ARP'able.
	 */

	if (in_dev == NULL)
		goto out_free_skb;

	arp = arp_hdr(skb); // Get ARP header from sk buff

	/*

	arp_project

	 Sanity check header fields based on the device type.
	 If failed drop packet.
	*/
	switch (dev_type) {
	default:
		if (arp->ar_pro != htons(ETH_P_IP) ||
		    htons(dev_type) != arp->ar_hrd)
			goto out_free_skb;
		break;
	case ARPHRD_ETHER:	// Ethernet is here
	case ARPHRD_FDDI:
	case ARPHRD_IEEE802:
		/*
		 * ETHERNET, and Fibre Channel (which are IEEE 802
		 * devices, according to RFC 2625) devices will accept ARP
		 * hardware types of either 1 (Ethernet) or 6 (IEEE 802.2).
		 * This is the case also of FDDI, where the RFC 1390 says that
		 * FDDI devices should accept ARP hardware of (1) Ethernet,
		 * however, to be more robust, we'll accept both 1 (Ethernet)
		 * or 6 (IEEE 802.2)
		 */
		if ((arp->ar_hrd != htons(ARPHRD_ETHER) &&
		     arp->ar_hrd != htons(ARPHRD_IEEE802)) ||
		    arp->ar_pro != htons(ETH_P_IP))
			goto out_free_skb;
		break;
	case ARPHRD_AX25:
		if (arp->ar_pro != htons(AX25_P_IP) ||
		    arp->ar_hrd != htons(ARPHRD_AX25))
			goto out_free_skb;
		break;
	case ARPHRD_NETROM:
		if (arp->ar_pro != htons(AX25_P_IP) ||
		    arp->ar_hrd != htons(ARPHRD_NETROM))
			goto out_free_skb;
		break;
	}

	/* Understand only these message types */

	if (arp->ar_op != htons(ARPOP_REPLY) &&
	    arp->ar_op != htons(ARPOP_REQUEST))
		goto out_free_skb;

	/* arp_project - Print arp_ptr infos */
	if (arp_project_enable && print_arp_info)
		arp_print_info(dev, arp, 0);

/*
 *	Extract fields
 */
	arp_ptr = (unsigned char *)(arp + 1);
	sha	= arp_ptr;
	arp_ptr += dev->addr_len;
	memcpy(&sip, arp_ptr, 4);
	arp_ptr += 4;

	switch (dev_type) {
#if IS_ENABLED(CONFIG_FIREWIRE_NET)
	case ARPHRD_IEEE1394:
		break;
#endif
	default:	// Ethernet is here
		/*

		arp_project

		 We can set ARP reply's sender HW adress by dev->dev_addr.
		 So we don't need target HW address from ARP request. Jump it.
		*/
		arp_ptr += dev->addr_len;
	}
	memcpy(&tip, arp_ptr, 4); // Get target IP address
/*
 *	Check for bad requests for 127.x.x.x and requests for multicast
 *	addresses.  If this is one such, delete it.
 */
	/*

	arp_project

	 If tip is loopback or multicast?
	 Yes -> Drop packet
	*/
	if (ipv4_is_multicast(tip) ||
	    (!IN_DEV_ROUTE_LOCALNET(in_dev) && ipv4_is_loopback(tip)))
		goto out_free_skb;

 /*
  *	For some 802.11 wireless deployments (and possibly other networks),
  *	there will be an ARP proxy and gratuitous ARP frames are attacks
  *	and thus should not be accepted.
  */
	if (sip == tip && IN_DEV_ORCONF(in_dev, DROP_GRATUITOUS_ARP))
		goto out_free_skb;

/*
 *     Special case: We must set Frame Relay source Q.922 address
 */
	if (dev_type == ARPHRD_DLCI)
		sha = dev->broadcast;

/*
 *  Process entry.  The idea here is we want to send a reply if it is a
 *  request for us or if it is a request for someone else that we hold
 *  a proxy for.  We want to add an entry to our cache if it is a reply
 *  to us or if it is a request for our address.
 *  (The assumption for this last is that if someone is requesting our
 *  address, they are probably intending to talk to us, so it saves time
 *  if we cache their address.  Their address is also probably not in
 *  our cache, since ours is not in their cache.)
 *
 *  Putting this another way, we only care about replies if they are to
 *  us, in which case we add them to the cache.  For requests, we care
 *  about those for us and those for our proxies.  We reply to both,
 *  and in the case of requests for us we add the requester to the arp
 *  cache.
 */

	/* Special case: IPv4 duplicate address detection packet (RFC2131) */
	/*

	arp_project

	 This is special case for detect duplicated IP address with DHCP.
	 DHCP server or DHCP client can send the ARP_REQUEST message with
	sip 0.0.0.0 and tip to use for that client.
	 If ARP_REPLY arrived, its IP is duplicated.

	 sip = 0.0.0.0?
	 Yes -> Is ARP_REQUEST? -> Is tip Local? -> Send ARP_REPLY
	 No -> Drop the packet
	*/
	if (sip == 0) {
		if (arp->ar_op == htons(ARPOP_REQUEST) &&
		    inet_addr_type(net, tip) == RTN_LOCAL &&
		    !arp_ignore(in_dev, sip, tip))
			arp_send(ARPOP_REPLY, ETH_P_ARP, sip, dev, tip, sha,
				 dev->dev_addr, sha);
		goto out_consume_skb;
	}

	if (arp->ar_op == htons(ARPOP_REQUEST) && // If REQUEST
	    ip_route_input_noref(skb, tip, sip, 0, dev) == 0) { // Is there a route between sip & tip?

		rt = skb_rtable(skb);
		addr_type = rt->rt_type;

		if (addr_type == RTN_LOCAL) { // Is tip local address?
			int dont_send;

			// ARP filter or ignore?
			dont_send = arp_ignore(in_dev, sip, tip);
			if (!dont_send && IN_DEV_ARPFILTER(in_dev))
				dont_send = arp_filter(sip, tip, dev);
			if (!dont_send) {
				/*
				 * arp_project
				 *
				 *  Find default gateway from route table and
				 * ignore updates when hardware address is different.
				 */
				if (arp_project_enable && ignore_gw_update_by_request) {
					if (arp_detect_gw_update(dev, sip, sha)) {
						printk(ARP_PROJECT"%s: "
						       "Ignoring ARP request...\n",
						       __func__);
						goto out_free_skb;
					}
				}

				// Is there already a neighbour entry for sip?
				// No -> Create it.
				// Yes -> Update it.
				// Return created or updated neigh if succes
				n = neigh_event_ns(&arp_tbl, sha, &sip, dev);
				if (n) {
					// Send ARP reply
					arp_send(ARPOP_REPLY, ETH_P_ARP, sip,
						 dev, tip, sha, dev->dev_addr,
						 sha);
					neigh_release(n);
				}
			}
			goto out_consume_skb; // End
		} else if (IN_DEV_FORWARD(in_dev)) { // Is IPv4 forwarding enabled?
			// Proxy ARP
			if (addr_type == RTN_UNICAST  &&
			    (arp_fwd_proxy(in_dev, dev, rt) ||
			     arp_fwd_pvlan(in_dev, dev, rt, sip, tip) ||
			     (rt->dst.dev != dev &&
			      pneigh_lookup(&arp_tbl, net, &tip, dev, 0)))) { // Is the request entry present in the proxy ARP table?
				/*
				 * arp_project
				 *
				 *  Find default gateway from route table and
				 * ignore updates when hardware address is different.
				 */
				if (arp_project_enable && ignore_gw_update_by_request) {
					if (arp_detect_gw_update(dev, sip, sha)) {
						printk(ARP_PROJECT"%s: "
						       "Ignoring ARP request...\n",
						       __func__);
						goto out_free_skb;
					}
				}

				// Is there already a neibour entry for sip?
				// Create or update
				n = neigh_event_ns(&arp_tbl, sha, &sip, dev);
				if (n)
					neigh_release(n);

				if (NEIGH_CB(skb)->flags & LOCALLY_ENQUEUED ||
				    skb->pkt_type == PACKET_HOST ||
				    in_dev->arp_parms->proxy_delay == 0) {
					arp_send(ARPOP_REPLY, ETH_P_ARP, sip,
						 dev, tip, sha, dev->dev_addr,
						 sha);
				} else {
					pneigh_enqueue(&arp_tbl,
						       in_dev->arp_parms, skb);
					return 0;
				}
				goto out_consume_skb; // End
			}
		}
	}

	// If REPLY

	if (arp_project_enable && ignore_gw_update_by_reply) {
		if (arp->ar_op == htons(ARPOP_REPLY)) {
			if (arp_detect_gw_update(dev, sip, sha)) {
				printk(ARP_PROJECT"%s: "
				       "Ignoring ARP reply...\n",
				       __func__);
				goto out_free_skb;
			}
		}
	}

	/* Update our ARP tables */

	// Is there already a neibour entry for sip?
	n = __neigh_lookup(&arp_tbl, &sip, dev, 0);
/////////////////// Reference /////////////////////
/*
static inline struct neighbour *
__neigh_lookup(struct neigh_table *tbl, const void *pkey, struct net_device *dev, int creat)
{
	struct neighbour *n = neigh_lookup(tbl, pkey, dev);

	if (n || !creat)
		return n;

	n = neigh_create(tbl, pkey, dev);
	return IS_ERR(n) ? NULL : n;
}

static inline struct neighbour *neigh_create(struct neigh_table *tbl,
					     const void *pkey,
					     struct net_device *dev)
{
	return __neigh_create(tbl, pkey, dev, true);
}
*/
///////////////////////////////////////////////////

	if (IN_DEV_ARP_ACCEPT(in_dev)) {
		/* Unsolicited ARP is not accepted by default.
		   It is possible, that this option should be enabled for some
		   devices (strip is candidate)
		 */
		/*
		arp_project

		GARP (Gratuitous ARP) - Send ARP request with sip == tip

		 When ARP request sended with same source IP and target IP,
		received hosts will update there ARP tables with new hardware address.

		 Also GARP used to find duplicated IP address.
		 It will receive ARP reply when there is conflicted IP address.
		*/
		is_garp = arp->ar_op == htons(ARPOP_REQUEST) && tip == sip &&
			  inet_addr_type(net, sip) == RTN_UNICAST;

		if (n == NULL &&
		    ((arp->ar_op == htons(ARPOP_REPLY)  &&
		      inet_addr_type(net, sip) == RTN_UNICAST) || is_garp))
			n = __neigh_lookup(&arp_tbl, &sip, dev, 1);
	}

	/*

	arp_project

	  Check NUD states here.
	  http://www.embeddedlinux.org.cn/linux_net/0596002556/understandlni-CHP-26-SECT-6.html
	*/
	if (n) {
		int state = NUD_REACHABLE;
		int override;

		// Is the last update older then locktime?
		// No -> end
		// Yes -> Update entry and set state to NUD_STALE

		/* If several different ARP replies follows back-to-back,
		   use the FIRST one. It is possible, if several proxy
		   agents are active. Taking the first reply prevents
		   arp trashing and chooses the fastest router.
		 */
		override = time_after(jiffies,
				      n->updated + n->parms->locktime) ||
			   is_garp;

		/* Broadcast replies and request packets
		   do not assert neighbour reachability.
		 */
		if (arp->ar_op != htons(ARPOP_REPLY) ||
		    skb->pkt_type != PACKET_HOST)
			state = NUD_STALE;
		/* arp_project - See net/core/neighbour.c */
		neigh_update(n, sha, state,
			     override ? NEIGH_UPDATE_F_OVERRIDE : 0);
		neigh_release(n);
	} // End

out_consume_skb:
	consume_skb(skb); // free an skbuff
	return NET_RX_SUCCESS;

out_free_skb:
	kfree_skb(skb);
	return NET_RX_DROP;
}

static void parp_redo(struct sk_buff *skb)
{
	arp_process(skb);
}


/*
 *	Receive an arp request from the device layer.
 */

static int arp_rcv(struct sk_buff *skb, struct net_device *dev,
		   struct packet_type *pt, struct net_device *orig_dev)
{
	const struct arphdr *arp;

	/* do not tweak dropwatch on an ARP we will ignore */
	if (dev->flags & IFF_NOARP ||
	    skb->pkt_type == PACKET_OTHERHOST ||
	    skb->pkt_type == PACKET_LOOPBACK)
		goto consumeskb;

	skb = skb_share_check(skb, GFP_ATOMIC);
	if (!skb)
		goto out_of_mem;

	/* ARP header, plus 2 device addresses, plus 2 IP addresses.  */
	if (!pskb_may_pull(skb, arp_hdr_len(dev)))
		goto freeskb;

	arp = arp_hdr(skb);
	if (arp->ar_hln != dev->addr_len || arp->ar_pln != 4)
		goto freeskb;

	memset(NEIGH_CB(skb), 0, sizeof(struct neighbour_cb));

	return NF_HOOK(NFPROTO_ARP, NF_ARP_IN, skb, dev, NULL, arp_process);

consumeskb:
	consume_skb(skb);
	return NET_RX_SUCCESS;
freeskb:
	kfree_skb(skb);
out_of_mem:
	return NET_RX_DROP;
}

/*
 *	User level interface (ioctl)
 */

/*
 *	Set (create) an ARP cache entry.
 */

static int arp_req_set_proxy(struct net *net, struct net_device *dev, int on)
{
	if (dev == NULL) {
		IPV4_DEVCONF_ALL(net, PROXY_ARP) = on;
		return 0;
	}
	if (__in_dev_get_rtnl(dev)) {
		IN_DEV_CONF_SET(__in_dev_get_rtnl(dev), PROXY_ARP, on);
		return 0;
	}
	return -ENXIO;
}

static int arp_req_set_public(struct net *net, struct arpreq *r,
		struct net_device *dev)
{
	__be32 ip = ((struct sockaddr_in *)&r->arp_pa)->sin_addr.s_addr;
	__be32 mask = ((struct sockaddr_in *)&r->arp_netmask)->sin_addr.s_addr;

	if (mask && mask != htonl(0xFFFFFFFF))
		return -EINVAL;
	if (!dev && (r->arp_flags & ATF_COM)) {
		dev = dev_getbyhwaddr_rcu(net, r->arp_ha.sa_family,
				      r->arp_ha.sa_data);
		if (!dev)
			return -ENODEV;
	}
	if (mask) {
		if (pneigh_lookup(&arp_tbl, net, &ip, dev, 1) == NULL)
			return -ENOBUFS;
		return 0;
	}

	return arp_req_set_proxy(net, dev, 1);
}

static int arp_req_set(struct net *net, struct arpreq *r,
		       struct net_device *dev)
{
	__be32 ip;
	struct neighbour *neigh;
	int err;

	if (r->arp_flags & ATF_PUBL)
		return arp_req_set_public(net, r, dev);

	ip = ((struct sockaddr_in *)&r->arp_pa)->sin_addr.s_addr;
	if (r->arp_flags & ATF_PERM)
		r->arp_flags |= ATF_COM;
	if (dev == NULL) {
		struct rtable *rt = ip_route_output(net, ip, 0, RTO_ONLINK, 0);

		if (IS_ERR(rt))
			return PTR_ERR(rt);
		dev = rt->dst.dev;
		ip_rt_put(rt);
		if (!dev)
			return -EINVAL;
	}
	switch (dev->type) {
#if IS_ENABLED(CONFIG_FDDI)
	case ARPHRD_FDDI:
		/*
		 * According to RFC 1390, FDDI devices should accept ARP
		 * hardware types of 1 (Ethernet).  However, to be more
		 * robust, we'll accept hardware types of either 1 (Ethernet)
		 * or 6 (IEEE 802.2).
		 */
		if (r->arp_ha.sa_family != ARPHRD_FDDI &&
		    r->arp_ha.sa_family != ARPHRD_ETHER &&
		    r->arp_ha.sa_family != ARPHRD_IEEE802)
			return -EINVAL;
		break;
#endif
	default:
		if (r->arp_ha.sa_family != dev->type)
			return -EINVAL;
		break;
	}

	neigh = __neigh_lookup_errno(&arp_tbl, &ip, dev);
	err = PTR_ERR(neigh);
	if (!IS_ERR(neigh)) {
		unsigned int state = NUD_STALE;
		if (r->arp_flags & ATF_PERM)
			state = NUD_PERMANENT;
		err = neigh_update(neigh, (r->arp_flags & ATF_COM) ?
				   r->arp_ha.sa_data : NULL, state,
				   NEIGH_UPDATE_F_OVERRIDE |
				   NEIGH_UPDATE_F_ADMIN);
		neigh_release(neigh);
	}
	return err;
}

static unsigned int arp_state_to_flags(struct neighbour *neigh)
{
	if (neigh->nud_state&NUD_PERMANENT)
		return ATF_PERM | ATF_COM;
	else if (neigh->nud_state&NUD_VALID)
		return ATF_COM;
	else
		return 0;
}

/*
 *	Get an ARP cache entry.
 */

static int arp_req_get(struct arpreq *r, struct net_device *dev)
{
	__be32 ip = ((struct sockaddr_in *) &r->arp_pa)->sin_addr.s_addr;
	struct neighbour *neigh;
	int err = -ENXIO;

	neigh = neigh_lookup(&arp_tbl, &ip, dev);
	if (neigh) {
		if (!(neigh->nud_state & NUD_NOARP)) {
			read_lock_bh(&neigh->lock);
			memcpy(r->arp_ha.sa_data, neigh->ha, dev->addr_len);
			r->arp_flags = arp_state_to_flags(neigh);
			read_unlock_bh(&neigh->lock);
			r->arp_ha.sa_family = dev->type;
			strlcpy(r->arp_dev, dev->name, sizeof(r->arp_dev));
			err = 0;
		}
		neigh_release(neigh);
	}
	return err;
}

static int arp_invalidate(struct net_device *dev, __be32 ip)
{
	struct neighbour *neigh = neigh_lookup(&arp_tbl, &ip, dev);
	int err = -ENXIO;

	if (neigh) {
		if (neigh->nud_state & ~NUD_NOARP)
			err = neigh_update(neigh, NULL, NUD_FAILED,
					   NEIGH_UPDATE_F_OVERRIDE|
					   NEIGH_UPDATE_F_ADMIN);
		neigh_release(neigh);
	}

	return err;
}

static int arp_req_delete_public(struct net *net, struct arpreq *r,
		struct net_device *dev)
{
	__be32 ip = ((struct sockaddr_in *) &r->arp_pa)->sin_addr.s_addr;
	__be32 mask = ((struct sockaddr_in *)&r->arp_netmask)->sin_addr.s_addr;

	if (mask == htonl(0xFFFFFFFF))
		return pneigh_delete(&arp_tbl, net, &ip, dev);

	if (mask)
		return -EINVAL;

	return arp_req_set_proxy(net, dev, 0);
}

static int arp_req_delete(struct net *net, struct arpreq *r,
			  struct net_device *dev)
{
	__be32 ip;

	if (r->arp_flags & ATF_PUBL)
		return arp_req_delete_public(net, r, dev);

	ip = ((struct sockaddr_in *)&r->arp_pa)->sin_addr.s_addr;
	if (dev == NULL) {
		struct rtable *rt = ip_route_output(net, ip, 0, RTO_ONLINK, 0);
		if (IS_ERR(rt))
			return PTR_ERR(rt);
		dev = rt->dst.dev;
		ip_rt_put(rt);
		if (!dev)
			return -EINVAL;
	}
	return arp_invalidate(dev, ip);
}

/*
 *	Handle an ARP layer I/O control request.
 */

int arp_ioctl(struct net *net, unsigned int cmd, void __user *arg)
{
	int err;
	struct arpreq r;
	struct net_device *dev = NULL;

	switch (cmd) {
	case SIOCDARP:
	case SIOCSARP:
		if (!ns_capable(net->user_ns, CAP_NET_ADMIN))
			return -EPERM;
	case SIOCGARP:
		err = copy_from_user(&r, arg, sizeof(struct arpreq));
		if (err)
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}

	if (r.arp_pa.sa_family != AF_INET)
		return -EPFNOSUPPORT;

	if (!(r.arp_flags & ATF_PUBL) &&
	    (r.arp_flags & (ATF_NETMASK | ATF_DONTPUB)))
		return -EINVAL;
	if (!(r.arp_flags & ATF_NETMASK))
		((struct sockaddr_in *)&r.arp_netmask)->sin_addr.s_addr =
							   htonl(0xFFFFFFFFUL);
	rtnl_lock();
	if (r.arp_dev[0]) {
		err = -ENODEV;
		dev = __dev_get_by_name(net, r.arp_dev);
		if (dev == NULL)
			goto out;

		/* Mmmm... It is wrong... ARPHRD_NETROM==0 */
		if (!r.arp_ha.sa_family)
			r.arp_ha.sa_family = dev->type;
		err = -EINVAL;
		if ((r.arp_flags & ATF_COM) && r.arp_ha.sa_family != dev->type)
			goto out;
	} else if (cmd == SIOCGARP) {
		err = -ENODEV;
		goto out;
	}

	switch (cmd) {
	case SIOCDARP:
		err = arp_req_delete(net, &r, dev);
		break;
	case SIOCSARP:
		err = arp_req_set(net, &r, dev);
		break;
	case SIOCGARP:
		err = arp_req_get(&r, dev);
		break;
	}
out:
	rtnl_unlock();
	if (cmd == SIOCGARP && !err && copy_to_user(arg, &r, sizeof(r)))
		err = -EFAULT;
	return err;
}

static int arp_netdev_event(struct notifier_block *this, unsigned long event,
			    void *ptr)
{
	struct net_device *dev = ptr;

	switch (event) {
	case NETDEV_CHANGEADDR:
		neigh_changeaddr(&arp_tbl, dev);
		rt_cache_flush(dev_net(dev));
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block arp_netdev_notifier = {
	.notifier_call = arp_netdev_event,
};

/* Note, that it is not on notifier chain.
   It is necessary, that this routine was called after route cache will be
   flushed.
 */
void arp_ifdown(struct net_device *dev)
{
	neigh_ifdown(&arp_tbl, dev);
}


/*
 *	Called once on startup.
 */

static struct packet_type arp_packet_type __read_mostly = {
	.type =	cpu_to_be16(ETH_P_ARP),
	.func =	arp_rcv,
};

static int arp_proc_init(void);
static void arp_sys_init(void);

void __init arp_init(void)
{
	neigh_table_init(&arp_tbl);

	dev_add_pack(&arp_packet_type);
	arp_proc_init();

	/* arp_project */
	printk("(C) 2016 arp_project by jollaman999, hasuk58, peace7944\n");
	arp_sys_init();

#ifdef CONFIG_SYSCTL
	neigh_sysctl_register(NULL, &arp_tbl.parms, "ipv4", NULL);
#endif
	register_netdevice_notifier(&arp_netdev_notifier);
}

#ifdef CONFIG_PROC_FS
#if IS_ENABLED(CONFIG_AX25)

/* ------------------------------------------------------------------------ */
/*
 *	ax25 -> ASCII conversion
 */
static char *ax2asc2(ax25_address *a, char *buf)
{
	char c, *s;
	int n;

	for (n = 0, s = buf; n < 6; n++) {
		c = (a->ax25_call[n] >> 1) & 0x7F;

		if (c != ' ')
			*s++ = c;
	}

	*s++ = '-';
	n = (a->ax25_call[6] >> 1) & 0x0F;
	if (n > 9) {
		*s++ = '1';
		n -= 10;
	}

	*s++ = n + '0';
	*s++ = '\0';

	if (*buf == '\0' || *buf == '-')
		return "*";

	return buf;
}
#endif /* CONFIG_AX25 */

#define HBUFFERLEN 30

static void arp_format_neigh_entry(struct seq_file *seq,
				   struct neighbour *n)
{
	char hbuffer[HBUFFERLEN];
	int k, j;
	char tbuf[16];
	struct net_device *dev = n->dev;
	int hatype = dev->type;

	read_lock(&n->lock);
	/* Convert hardware address to XX:XX:XX:XX ... form. */
#if IS_ENABLED(CONFIG_AX25)
	if (hatype == ARPHRD_AX25 || hatype == ARPHRD_NETROM)
		ax2asc2((ax25_address *)n->ha, hbuffer);
	else {
#endif
	for (k = 0, j = 0; k < HBUFFERLEN - 3 && j < dev->addr_len; j++) {
		hbuffer[k++] = hex_asc_hi(n->ha[j]);
		hbuffer[k++] = hex_asc_lo(n->ha[j]);
		hbuffer[k++] = ':';
	}
	if (k != 0)
		--k;
	hbuffer[k] = 0;
#if IS_ENABLED(CONFIG_AX25)
	}
#endif
	sprintf(tbuf, "%pI4", n->primary_key);
	seq_printf(seq, "%-16s 0x%-10x0x%-10x%s     *        %s\n",
		   tbuf, hatype, arp_state_to_flags(n), hbuffer, dev->name);
	read_unlock(&n->lock);
}

static void arp_format_pneigh_entry(struct seq_file *seq,
				    struct pneigh_entry *n)
{
	struct net_device *dev = n->dev;
	int hatype = dev ? dev->type : 0;
	char tbuf[16];

	sprintf(tbuf, "%pI4", n->key);
	seq_printf(seq, "%-16s 0x%-10x0x%-10x%s     *        %s\n",
		   tbuf, hatype, ATF_PUBL | ATF_PERM, "00:00:00:00:00:00",
		   dev ? dev->name : "*");
}

static int arp_seq_show(struct seq_file *seq, void *v)
{
	if (v == SEQ_START_TOKEN) {
		seq_puts(seq, "IP address       HW type     Flags       "
			      "HW address            Mask     Device\n");
	} else {
		struct neigh_seq_state *state = seq->private;

		if (state->flags & NEIGH_SEQ_IS_PNEIGH)
			arp_format_pneigh_entry(seq, v);
		else
			arp_format_neigh_entry(seq, v);
	}

	return 0;
}

static void *arp_seq_start(struct seq_file *seq, loff_t *pos)
{
	/* Don't want to confuse "arp -a" w/ magic entries,
	 * so we tell the generic iterator to skip NUD_NOARP.
	 */
	return neigh_seq_start(seq, pos, &arp_tbl, NEIGH_SEQ_SKIP_NOARP);
}

/* ------------------------------------------------------------------------ */

static const struct seq_operations arp_seq_ops = {
	.start	= arp_seq_start,
	.next	= neigh_seq_next,
	.stop	= neigh_seq_stop,
	.show	= arp_seq_show,
};

static int arp_seq_open(struct inode *inode, struct file *file)
{
	return seq_open_net(inode, file, &arp_seq_ops,
			    sizeof(struct neigh_seq_state));
}

static const struct file_operations arp_seq_fops = {
	.owner		= THIS_MODULE,
	.open           = arp_seq_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release	= seq_release_net,
};


static int __net_init arp_net_init(struct net *net)
{
	if (!proc_create("arp", S_IRUGO, net->proc_net, &arp_seq_fops))
		return -ENOMEM;
	return 0;
}

static void __net_exit arp_net_exit(struct net *net)
{
	remove_proc_entry("arp", net->proc_net);
}

static struct pernet_operations arp_net_ops = {
	.init = arp_net_init,
	.exit = arp_net_exit,
};

static int __init arp_proc_init(void)
{
	return register_pernet_subsys(&arp_net_ops);
}

#else /* CONFIG_PROC_FS */

static int __init arp_proc_init(void)
{
	return 0;
}

#endif /* CONFIG_PROC_FS */

/********************** arp_project sysfs **********************/
static ssize_t arp_project_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%s\n", ARP_PROJECT_VERSION);

	return count;
}

static ssize_t arp_project_version_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(arp_project_version, (S_IWUSR|S_IRUGO),
	arp_project_version_show, arp_project_version_dump);

static ssize_t arp_project_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", arp_project_enable);

	return count;
}

static ssize_t arp_project_enable_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (arp_project_enable)
		val = 1;

	if ((buf[0] == '0' || buf[0] == '1') && buf[1] == '\n') {
		if (val != buf[0] - '0')
			val = buf[0] - '0';
		else
			return count;
	} else
		return -EINVAL;

	if (val) {
		arp_project_enable = true;
		printk(ARP_PROJECT"%s: Enabled\n", __func__);
	} else {
		arp_project_enable = false;
		printk(ARP_PROJECT"%s: Disabled\n", __func__);
	}

	return count;
}

static DEVICE_ATTR(arp_project_enable, (S_IWUSR|S_IRUGO),
	arp_project_enable_show, arp_project_enable_dump);

static ssize_t print_arp_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", print_arp_info);

	return count;
}

static ssize_t print_arp_info_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (print_arp_info)
		val = 1;

	if ((buf[0] == '0' || buf[0] == '1') && buf[1] == '\n') {
		if (val != buf[0] - '0')
			val = buf[0] - '0';
		else
			return count;
	} else
		return -EINVAL;

	if (val) {
		print_arp_info = true;
		printk(ARP_PROJECT"%s: Enabled\n", __func__);
	} else {
		print_arp_info = false;
		printk(ARP_PROJECT"%s: Disabled\n", __func__);
	}

	return count;
}

static DEVICE_ATTR(print_arp_info, (S_IWUSR|S_IRUGO),
	print_arp_info_show, print_arp_info_dump);

static ssize_t ignore_gw_update_by_request_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", ignore_gw_update_by_request);

	return count;
}

static ssize_t ignore_gw_update_by_request_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (ignore_gw_update_by_request)
		val = 1;

	if ((buf[0] == '0' || buf[0] == '1') && buf[1] == '\n') {
		if (val != buf[0] - '0')
			val = buf[0] - '0';
		else
			return count;
	} else
		return -EINVAL;

	if (val) {
		ignore_gw_update_by_request = true;
		printk(ARP_PROJECT"%s: Enabled\n", __func__);
	} else {
		ignore_gw_update_by_request = false;
		printk(ARP_PROJECT"%s: Disabled\n", __func__);
	}

	return count;
}

static DEVICE_ATTR(ignore_gw_update_by_request, (S_IWUSR|S_IRUGO),
	ignore_gw_update_by_request_show, ignore_gw_update_by_request_dump);

static ssize_t ignore_gw_update_by_reply_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", ignore_gw_update_by_reply);

	return count;
}

static ssize_t ignore_gw_update_by_reply_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (ignore_gw_update_by_reply)
		val = 1;

	if ((buf[0] == '0' || buf[0] == '1') && buf[1] == '\n') {
		if (val != buf[0] - '0')
			val = buf[0] - '0';
		else
			return count;
	} else
		return -EINVAL;

	if (val) {
		ignore_gw_update_by_reply = true;
		printk(ARP_PROJECT"%s: Enabled\n", __func__);
	} else {
		ignore_gw_update_by_reply = false;
		printk(ARP_PROJECT"%s: Disabled\n", __func__);
	}

	return count;
}

static DEVICE_ATTR(ignore_gw_update_by_reply, (S_IWUSR|S_IRUGO),
	ignore_gw_update_by_reply_show, ignore_gw_update_by_reply_dump);

struct kobject *arp_project_kobj;

static void __init arp_sys_init(void)
{
	int rc;

	arp_project_kobj = kobject_create_and_add("arp_project", NULL);
	if (arp_project_kobj == NULL) {
		pr_warn("%s: arp_project_kobj create_and_add failed\n", __func__);
	}

	rc = sysfs_create_file(arp_project_kobj, &dev_attr_arp_project_version.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for arp_project_version\n", __func__);
	}

	rc = sysfs_create_file(arp_project_kobj, &dev_attr_arp_project_enable.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for arp_project_enable\n", __func__);
	}

	rc = sysfs_create_file(arp_project_kobj, &dev_attr_print_arp_info.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for print_arp_info\n", __func__);
	}

	rc = sysfs_create_file(arp_project_kobj, &dev_attr_ignore_gw_update_by_request.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for ignore_gw_update_by_request\n", __func__);
	}

	rc = sysfs_create_file(arp_project_kobj, &dev_attr_ignore_gw_update_by_reply.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for ignore_gw_update_by_reply\n", __func__);
	}
}
/********************** arp_project sysfs **********************/
