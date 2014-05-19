/*
 * AHCI SATA platform driver
 *
 * Copyright 2004-2005  Red Hat, Inc.
 *   Jeff Garzik <jgarzik@pobox.com>
 * Copyright 2010  MontaVista Software, LLC.
 *   Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include <linux/kernel.h>
#include <linux/gfp.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/libata.h>
#include <linux/ahci_platform.h>
#if defined(CONFIG_SYNO_COMCERTO)
#include <linux/clk.h>
#include <mach/reset.h>
#endif
#include "ahci.h"

#if defined(CONFIG_SYNO_COMCERTO) && defined(CONFIG_ARCH_M86XXX)
/* SATA Clocks */
static struct clk *sata_oob_clk; /* Core clock */
static struct clk *sata_pmu_clk; /* PMU alive clock */
static struct clk *sata_clk;	/* Sata AXI ref clock */
#endif 

enum ahci_type {
	AHCI,		/* standard platform ahci */
	IMX53_AHCI,	/* ahci on i.mx53 */
};

static struct platform_device_id ahci_devtype[] = {
	{
		.name = "ahci",
		.driver_data = AHCI,
	}, {
		.name = "imx53-ahci",
		.driver_data = IMX53_AHCI,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, ahci_devtype);


static const struct ata_port_info ahci_port_info[] = {
	/* by features */
	[AHCI] = {
		.flags		= AHCI_FLAG_COMMON,
		.pio_mask	= ATA_PIO4,
		.udma_mask	= ATA_UDMA6,
		.port_ops	= &ahci_ops,
	},
	[IMX53_AHCI] = {
		.flags		= AHCI_FLAG_COMMON,
		.pio_mask	= ATA_PIO4,
		.udma_mask	= ATA_UDMA6,
		.port_ops	= &ahci_pmp_retry_srst_ops,
	},
};

static struct scsi_host_template ahci_platform_sht = {
	AHCI_SHT("ahci_platform"),
};

#if defined(CONFIG_SYNO_COMCERTO) && defined(CONFIG_PM)
static int ahci_platform_suspend(struct platform_device *pdev, pm_message_t state)
{
        struct ata_host *host = platform_get_drvdata(pdev);
	int ret=0;
        if (host)
		ret = ata_host_suspend(host, state);

#ifdef CONFIG_ARCH_M86XXX
	if (!ret) /* sucessfully done the host suspend */
	{
		/* No do the clock disable PMU,OOB,AXI here */
		clk_disable(sata_clk);
		clk_disable(sata_oob_clk);
		clk_disable(sata_pmu_clk);
	}
#endif
	
        return ret;
}

static int ahci_platform_resume(struct platform_device *pdev)
{
        struct ata_host *host = platform_get_drvdata(pdev);

#ifdef CONFIG_ARCH_M86XXX
	/* Do the  clock enable here  PMU,OOB,AXI */
	clk_enable(sata_clk);
	clk_enable(sata_oob_clk);
	clk_enable(sata_pmu_clk);
#endif

        if (host) 
		ata_host_resume(host);

	return 0;
}
#else
#define ahci_platform_suspend NULL
#define ahci_platform_resume NULL
#endif



static int __init ahci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ahci_platform_data *pdata = dev_get_platdata(dev);
	const struct platform_device_id *id = platform_get_device_id(pdev);
	struct ata_port_info pi = ahci_port_info[id ? id->driver_data : 0];
	const struct ata_port_info *ppi[] = { &pi, NULL };
	struct ahci_host_priv *hpriv;
	struct ata_host *host;
	struct resource *mem;
	int irq;
	int n_ports;
	int i;
	int rc;
#if defined(CONFIG_SYNO_COMCERTO) && defined(CONFIG_ARCH_M86XXX)
	/* Get the Reference and Enable  the SATA clocks here */

	sata_clk = clk_get(NULL,"sata");
	/* Error Handling , if no SATA(AXI) clock reference: return error */
	if (IS_ERR(sata_clk)) {
		pr_err("%s: Unable to obtain SATA(AXI) clock: %ld\n",__func__,PTR_ERR(sata_clk));
		return PTR_ERR(sata_clk);
 	}

	/*Enable the SATA(AXI) clock here */
        rc = clk_enable(sata_clk);
	if (rc){
		pr_err("%s: SATA(AXI) clock enable failed \n",__func__);
                return rc;
	}
	sata_oob_clk = clk_get(NULL,"sata_oob");
	/* Error Handling , if no SATA_OOB clock reference: return error */
	if (IS_ERR(sata_oob_clk)) {
		pr_err("%s: Unable to obtain SATA_OOB clock: %ld\n",__func__,PTR_ERR(sata_oob_clk));
		return PTR_ERR(sata_oob_clk);
 	}

	sata_pmu_clk = clk_get(NULL,"sata_pmu");
	/* Error Handling , if no SATA_PMU clock reference: return error */
	if (IS_ERR(sata_pmu_clk)) {
		pr_err("%s: Unable to obtain SATA_PMU clock: %ld\n",__func__,PTR_ERR(sata_pmu_clk));
		return PTR_ERR(sata_pmu_clk);
	}
	/*Enable the SATA(PMU and OOB) clocks here */
        rc = clk_enable(sata_oob_clk);
	if (rc){
		pr_err("%s: SATA_OOB clock enable failed \n",__func__);
                return rc;
	}

        rc = clk_enable(sata_pmu_clk);
	if (rc){
		pr_err("%s: SATA_PMU clock enable failed \n",__func__);
		return rc;
	}

	/* Set the SATA PMU clock to 30 MHZ and OOB clock to 125MHZ */
	clk_set_rate(sata_oob_clk,125000000);
	clk_set_rate(sata_pmu_clk,30000000);
	
#endif

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "no mmio space\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(dev, "no irq\n");
		return -EINVAL;
	}

	if (pdata && pdata->ata_port_info)
		pi = *pdata->ata_port_info;

	hpriv = devm_kzalloc(dev, sizeof(*hpriv), GFP_KERNEL);
	if (!hpriv) {
		dev_err(dev, "can't alloc ahci_host_priv\n");
		return -ENOMEM;
	}

	hpriv->flags |= (unsigned long)pi.private_data;

	hpriv->mmio = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!hpriv->mmio) {
		dev_err(dev, "can't map %pR\n", mem);
		return -ENOMEM;
	}

	/*
	 * Some platforms might need to prepare for mmio region access,
	 * which could be done in the following init call. So, the mmio
	 * region shouldn't be accessed before init (if provided) has
	 * returned successfully.
	 */
	if (pdata && pdata->init) {
		rc = pdata->init(dev, hpriv->mmio);
		if (rc)
			return rc;
	}

	ahci_save_initial_config(dev, hpriv,
		pdata ? pdata->force_port_map : 0,
		pdata ? pdata->mask_port_map  : 0);

	/* prepare host */
	if (hpriv->cap & HOST_CAP_NCQ)
		pi.flags |= ATA_FLAG_NCQ;

	if (hpriv->cap & HOST_CAP_PMP)
		pi.flags |= ATA_FLAG_PMP;

	ahci_set_em_messages(hpriv, &pi);

	/* CAP.NP sometimes indicate the index of the last enabled
	 * port, at other times, that of the last possible port, so
	 * determining the maximum port number requires looking at
	 * both CAP.NP and port_map.
	 */
	n_ports = max(ahci_nr_ports(hpriv->cap), fls(hpriv->port_map));

	host = ata_host_alloc_pinfo(dev, ppi, n_ports);
	if (!host) {
		rc = -ENOMEM;
		goto err0;
	}

	host->private_data = hpriv;

	if (!(hpriv->cap & HOST_CAP_SSS) || ahci_ignore_sss)
		host->flags |= ATA_HOST_PARALLEL_SCAN;
	else
		printk(KERN_INFO "ahci: SSS flag set, parallel bus scan disabled\n");

	if (pi.flags & ATA_FLAG_EM)
		ahci_reset_em(host);

	for (i = 0; i < host->n_ports; i++) {
		struct ata_port *ap = host->ports[i];

		ata_port_desc(ap, "mmio %pR", mem);
		ata_port_desc(ap, "port 0x%x", 0x100 + ap->port_no * 0x80);

		/* set enclosure management message type */
		if (ap->flags & ATA_FLAG_EM)
			ap->em_message_type = hpriv->em_msg_type;

#if defined(CONFIG_SYNO_COMCERTO) && defined(CONFIG_ARCH_M86XXX)
		/* Optimized PFE/SATA DDR interaction,
		limit burst size of SATA controller */
		writel(0 , ahci_port_base(ap) + 0x70);
#endif

		/* disabled/not-implemented port */
		if (!(hpriv->port_map & (1 << i)))
			ap->ops = &ata_dummy_port_ops;
	}

	rc = ahci_reset_controller(host);
	if (rc)
		goto err0;

	ahci_init_controller(host);
	ahci_print_info(host, "platform");

	rc = ata_host_activate(host, irq, ahci_interrupt, IRQF_SHARED,
			       &ahci_platform_sht);
	if (rc)
		goto err0;

	return 0;
err0:
	if (pdata && pdata->exit)
		pdata->exit(dev);
	return rc;
}

static int __devexit ahci_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ahci_platform_data *pdata = dev_get_platdata(dev);
	struct ata_host *host = dev_get_drvdata(dev);

	ata_host_detach(host);

	if (pdata && pdata->exit)
		pdata->exit(dev);
#if defined(CONFIG_SYNO_COMCERTO) && defined(CONFIG_ARCH_M86XXX)
	/* Disbale the SATA clocks Here */
	clk_disable(sata_clk);
	clk_put(sata_clk);
	clk_disable(sata_oob_clk);
	clk_put(sata_oob_clk);
	clk_disable(sata_pmu_clk);
	clk_put(sata_pmu_clk);

	/*Putting  SATA in reset state 
	 * Sata axi clock domain in reset state
	 * Serdes 1/2 in reset state, this depends upon PCIE1 and SGMII 
         * sata 0/1 serdes controller in reset state
	*/
	c2000_block_reset(COMPONENT_AXI_SATA,1);

	c2000_block_reset(COMPONENT_SERDES1,1);
	c2000_block_reset(COMPONENT_SERDES_SATA0,1);

	c2000_block_reset(COMPONENT_SERDES2,1);
	c2000_block_reset(COMPONENT_SERDES_SATA1,1);
#endif

	return 0;
}

static const struct of_device_id ahci_of_match[] = {
	{ .compatible = "calxeda,hb-ahci", },
	{},
};
MODULE_DEVICE_TABLE(of, ahci_of_match);

static struct platform_driver ahci_driver = {
	.remove  = __devexit_p(ahci_remove),
#if defined(CONFIG_SYNO_COMCERTO) && defined(CONFIG_PM)
	.suspend = ahci_platform_suspend,
	.resume  = ahci_platform_resume,
#endif
	.driver  = {
		 .name = "ahci",
		 .owner = THIS_MODULE,
		 .of_match_table = ahci_of_match,
	},
	.id_table = ahci_devtype,
};

static int __init ahci_init(void)
{
	return platform_driver_probe(&ahci_driver, ahci_probe);
}
module_init(ahci_init);

static void __exit ahci_exit(void)
{
	platform_driver_unregister(&ahci_driver);
}
module_exit(ahci_exit);

MODULE_DESCRIPTION("AHCI SATA platform driver");
MODULE_AUTHOR("Anton Vorontsov <avorontsov@ru.mvista.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ahci");
