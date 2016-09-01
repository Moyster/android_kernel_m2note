/* linux/net/arp_project.h */
#ifndef _ARP_PROJECT_H
#define _ARP_PROJECT_H

#define ARP_PROJECT "arp_project: "
#define ARP_PROJECT_VERSION "0.1"

bool arp_project_enable;
bool print_arp_info;
bool ignore_gw_update_by_request;

void arp_print_info(struct net_device *dev, struct arphdr *arp, int count);
void arp_print_and_check_send(struct net_device *dev, struct sk_buff *skb);

#endif	/* _ARP_PROJECT_H */
