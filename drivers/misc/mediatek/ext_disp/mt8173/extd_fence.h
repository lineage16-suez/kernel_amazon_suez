/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __EXTD_FENCE_H__
#define __EXTD_FENCE_H__

#include <linux/mutex.h>
#include <linux/list.h>

#ifdef __cplusplus
extern "C" {
#endif


#define MTK_FB_INVALID_ION_FD	   (-1)
#define MTK_FB_INVALID_FENCE_FD    (-1)

	struct fb_overlay_buffer_t {
		/* Input */
		int layer_id;
		unsigned int layer_en;
		int ion_fd;
		unsigned int cache_sync;
		/* Output */
		unsigned int index;
		int fence_fd;
	};

	struct extd_fence_buf_info {
		struct list_head list;
		unsigned int idx;
		int fence;
		struct ion_handle *hnd;
		unsigned int mva;
		unsigned int cache_sync;
	};
	struct extd_fence_sync_info {
		struct mutex mutex_lock;
		unsigned int layer_id;
		unsigned int fence_idx;
		unsigned int timeline_idx;
		unsigned int inc;
		unsigned int cur_idx;
		struct list_head buf_list;
	};

	void extd_init_fence(void);
	unsigned int extd_query_buf_mva(unsigned int layer, unsigned int idx);
	struct extd_fence_buf_info *extd_init_buf_info(struct extd_fence_buf_info *buf);
	void extd_release_fence(unsigned int layer, int fence);
	void extd_release_layer_fence(unsigned int layer);
	struct extd_fence_buf_info *extd_prepare_buf(struct fb_overlay_buffer_t *buf);
	int extd_fence_clean_thread(void *data);
	int extd_fence_timeline_index(unsigned int layer);
	unsigned int extd_inc_fence_index(unsigned int layer);
#ifdef __cplusplus
}
#endif
#endif
