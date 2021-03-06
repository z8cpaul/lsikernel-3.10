 #define AXM55XX_R1

#define AXM_55XX_PLATFORM_BASE 0x10000
#define AXM_55XX_VP_BASE (AXM_55XX_PLATFORM_BASE + 0x00)
#define AXM_55XX_VP_MAX (AXM_55XX_VP_BASE + 0x1fff)
#define AXM_55XX_PCX_BASE (AXM_55XX_PLATFORM_BASE + 0x4000)
#define AXM_55XX_PCX_MAX (AXM_55XX_PCX_BASE + 0x0fff)
#define AXM_55XX_MEMC_BASE (AXM_55XX_PLATFORM_BASE + 0x8000)
#define AXM_55XX_MEMC_MAX (AXM_55XX_MEMC_BASE + 0x0fff)
#define AXM_55XX_PLATFORM_MAX (AXM_55XX_MEMC_MAX)
