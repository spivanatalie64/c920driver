// c920virtual.c — OUR OWN virtual camera driver. No v4l2 framework.
// ------------------------------------------------------------------
//   hardware (C920) → our userspace pipeline → /dev/c920feed
//        → THIS DRIVER → /dev/video10 (raw character device that speaks
//          the V4L2 wire protocol itself: BGR3 only)
//        → Discord / Firefox / Chrome
//
// Everything here is ours: cdev registration, ioctl decoding, mmap,
// poll, the frame ring. The only thing we borrow is the videodev2.h
// header (the wire protocol apps speak).
//
// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/miscdevice.h>
#include <linux/videodev2.h>

#define DRV_NAME "c920virtual"
#define FEED_NAME "c920feed"
#define CAM_NODE 10                 /* /dev/video10 */

#define MAX_W 1920
#define MAX_H 1080

/* --------------------------- shared state --------------------------- */
static DEFINE_MUTEX(cam_lock);
static struct v4l2_pix_format cam_fmt = {
    .width        = 1280,
    .height       = 720,
    .pixelformat  = V4L2_PIX_FMT_BGR24,
    .field        = V4L2_FIELD_NONE,
    .bytesperline = 1280 * 3,
    .sizeimage    = 1280 * 720 * 3,
    .colorspace   = V4L2_COLORSPACE_SRGB,
};
static void *frame_buf;             /* vmalloc'ed, w*h*3 */
static u64   frame_seq;
static int   have_frame;
static int   buf_count = 1;   /* buffers granted to readers (max 4) */
static wait_queue_head_t frame_wait;

/* ------------------------ consumer /dev/video10 ---------------------- */
struct cam_fh {
    u64 seen_seq;
};

static int cam_open(struct inode *inode, struct file *f)
{
    struct cam_fh *fh = kzalloc(sizeof(*fh), GFP_KERNEL);
    if (!fh)
        return -ENOMEM;
    mutex_lock(&cam_lock);
    /* a frame already present is "new" to this reader — deliver it */
    fh->seen_seq = have_frame ? frame_seq - 1 : frame_seq;
    mutex_unlock(&cam_lock);
    f->private_data = fh;
    return 0;
}

static int cam_release(struct inode *inode, struct file *f)
{
    kfree(f->private_data);
    f->private_data = NULL;
    return 0;
}

static __poll_t cam_poll(struct file *f, poll_table *wait)
{
    struct cam_fh *fh = f->private_data;
    __poll_t mask = 0;
    poll_wait(f, &frame_wait, wait);
    mutex_lock(&cam_lock);
    if (have_frame && frame_seq != fh->seen_seq)
        mask |= EPOLLIN | EPOLLRDNORM;
    mutex_unlock(&cam_lock);
    return mask;
}

static int cam_mmap(struct file *f, struct vm_area_struct *vma)
{
    unsigned long size = vma->vm_end - vma->vm_start;
    if (size != cam_fmt.sizeimage)
        return -EINVAL;
    vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);
    return remap_vmalloc_range(vma, frame_buf, 0);
}

/* ---------------------------- ioctls (ours) -------------------------- */
static long cam_ioctl(struct file *f, unsigned int cmd, unsigned long arg);

static long cam_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    struct cam_fh *fh = f->private_data;
    void __user *u = (void __user *)arg;
    unsigned int nr = _IOC_NR(cmd);
    int rc = 0;

    if (_IOC_TYPE(cmd) != 'V')
        return -ENOIOCTLCMD;

    switch (nr) {
    case 0: {                       /* VIDIOC_QUERYCAP */
        struct v4l2_capability cap;
        memset(&cap, 0, sizeof(cap));
        strscpy(cap.driver, DRV_NAME, sizeof(cap.driver));
        strscpy(cap.card, "C920 HDR (ours)", sizeof(cap.card));
        strscpy(cap.bus_info, "platform:ours", sizeof(cap.bus_info));
        cap.capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING
                           | V4L2_CAP_DEVICE_CAPS;
        cap.device_caps  = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
        if (copy_to_user(u, &cap, sizeof(cap)))
            return -EFAULT;
        return 0;
    }
    case 2: {                       /* VIDIOC_ENUM_FMT */
        struct v4l2_fmtdesc fd;
        if (copy_from_user(&fd, u, sizeof(fd)))
            return -EFAULT;
        if (fd.index != 0) {
            rc = -EINVAL;
            break;
        }
        memset(&fd, 0, sizeof(fd));
        fd.index = 0;
        fd.pixelformat = V4L2_PIX_FMT_BGR24;
        strscpy(fd.description, "24-bit BGR 8-8-8", sizeof(fd.description));
        if (copy_to_user(u, &fd, sizeof(fd)))
            return -EFAULT;
        return 0;
    }
    case 74: {                      /* VIDIOC_ENUM_FRAMESIZES */
        struct v4l2_frmsizeenum fs;
        if (copy_from_user(&fs, u, sizeof(fs)))
            return -EFAULT;
        if (fs.index != 0 || fs.pixel_format != V4L2_PIX_FMT_BGR24) {
            rc = -EINVAL;
            break;
        }
        memset(&fs, 0, sizeof(fs));
        fs.index = 0;
        fs.pixel_format = V4L2_PIX_FMT_BGR24;
        fs.type = V4L2_FRMSIZE_TYPE_DISCRETE;
        fs.discrete.width  = cam_fmt.width;
        fs.discrete.height = cam_fmt.height;
        if (copy_to_user(u, &fs, sizeof(fs)))
            return -EFAULT;
        return 0;
    }
    case 4:                         /* VIDIOC_G_FMT */
    case 64: {                      /* VIDIOC_TRY_FMT */
        struct v4l2_format fmt;
        if (copy_from_user(&fmt, u, sizeof(fmt)))
            return -EFAULT;
        if (fmt.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            rc = -EINVAL;
            break;
        }
        fmt.fmt.pix = cam_fmt;
        if (copy_to_user(u, &fmt, sizeof(fmt)))
            return -EFAULT;
        return 0;
    }
    case 5: {                       /* VIDIOC_S_FMT */
        struct v4l2_format fmt;
        if (copy_from_user(&fmt, u, sizeof(fmt)))
            return -EFAULT;
        if (fmt.type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
            fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_BGR24) {
            rc = -EINVAL;
            break;
        }
        mutex_lock(&cam_lock);
        cam_fmt.width  = clamp_t(__u32, fmt.fmt.pix.width,  16, MAX_W);
        cam_fmt.height = clamp_t(__u32, fmt.fmt.pix.height, 16, MAX_H);
        cam_fmt.bytesperline = cam_fmt.width * 3;
        cam_fmt.sizeimage    = cam_fmt.width * cam_fmt.height * 3;
        if (frame_buf)
            vfree(frame_buf);
        frame_buf = vmalloc_user(cam_fmt.sizeimage);
        have_frame = 0;
        fmt.fmt.pix = cam_fmt;
        mutex_unlock(&cam_lock);
        if (!frame_buf)
            return -ENOMEM;
        if (copy_to_user(u, &fmt, sizeof(fmt)))
            return -EFAULT;
        return 0;
    }
    case 8: {                       /* VIDIOC_REQBUFS */
        struct v4l2_requestbuffers rb;
        if (copy_from_user(&rb, u, sizeof(rb)))
            return -EFAULT;
        if (rb.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            rc = -EINVAL;
            break;
        }
        buf_count = clamp_t(__u32, rb.count, 1, 4);
        rb.count = buf_count;       /* share the one frame buffer */
        if (copy_to_user(u, &rb, sizeof(rb)))
            return -EFAULT;
        return 0;
    }
    case 92: {                      /* VIDIOC_CREATE_BUFS */
        struct v4l2_create_buffers cb;
        if (copy_from_user(&cb, u, sizeof(cb)))
            return -EFAULT;
        if (cb.format.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            rc = -EINVAL;
            break;
        }
        cb.index = 0;
        buf_count = clamp_t(__u32, cb.count, 1, 4);
        cb.count = buf_count;
        cb.format.fmt.pix = cam_fmt;
        if (copy_to_user(u, &cb, sizeof(cb)))
            return -EFAULT;
        return 0;
    }
    case 9: {                       /* VIDIOC_QUERYBUF */
        struct v4l2_buffer b;
        if (copy_from_user(&b, u, sizeof(b)))
            return -EFAULT;
        if (b.type != V4L2_BUF_TYPE_VIDEO_CAPTURE || b.index >= buf_count) {
            rc = -EINVAL;
            break;
        }
        memset(&b, 0, sizeof(b));
        b.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.index  = b.index;
        b.memory = V4L2_MEMORY_MMAP;
        b.length = cam_fmt.sizeimage;
        b.m.offset = 0;
        if (copy_to_user(u, &b, sizeof(b)))
            return -EFAULT;
        return 0;
    }
    case 15: {                      /* VIDIOC_QBUF — always queued */
        struct v4l2_buffer b;
        if (copy_from_user(&b, u, sizeof(b)))
            return -EFAULT;
        if (b.type != V4L2_BUF_TYPE_VIDEO_CAPTURE || b.index >= buf_count)
            return -EINVAL;
        return 0;
    }
    case 17: {                      /* VIDIOC_DQBUF */
        struct v4l2_buffer b;
        if (copy_from_user(&b, u, sizeof(b)))
            return -EFAULT;
        if (b.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            rc = -EINVAL;
            break;
        }
        for (;;) {
            mutex_lock(&cam_lock);
            if (have_frame && frame_seq != fh->seen_seq) {
                fh->seen_seq = frame_seq;
                mutex_unlock(&cam_lock);
                break;
            }
            mutex_unlock(&cam_lock);
            if (f->f_flags & O_NONBLOCK)
                return -EAGAIN;
            if (wait_event_interruptible(frame_wait,
                    have_frame && frame_seq != fh->seen_seq))
                return -ERESTARTSYS;
        }
        memset(&b, 0, sizeof(b));
        b.type      = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.index     = 0;
        b.memory    = V4L2_MEMORY_MMAP;
        b.bytesused = cam_fmt.sizeimage;
        b.sequence  = (__u32)frame_seq;
        if (copy_to_user(u, &b, sizeof(b)))
            return -EFAULT;
        return 0;
    }
    case 18:                        /* VIDIOC_STREAMON */
    case 19:                        /* VIDIOC_STREAMOFF */
        return 0;
    case 21:                        /* VIDIOC_G_PARM */
    case 22: {                      /* VIDIOC_S_PARM */
        struct v4l2_streamparm sp;
        if (copy_from_user(&sp, u, sizeof(sp)))
            return -EFAULT;
        if (sp.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            rc = -EINVAL;
            break;
        }
        sp.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
        sp.parm.capture.timeperframe.numerator = 1;
        sp.parm.capture.timeperframe.denominator = 30;
        if (copy_to_user(u, &sp, sizeof(sp)))
            return -EFAULT;
        return 0;
    }
    case 26: {                      /* VIDIOC_ENUMINPUT */
        struct v4l2_input inp;
        if (copy_from_user(&inp, u, sizeof(inp)))
            return -EFAULT;
        if (inp.index != 0) {
            rc = -EINVAL;
            break;
        }
        memset(&inp, 0, sizeof(inp));
        inp.index = 0;
        strscpy(inp.name, "Camera", sizeof(inp.name));
        inp.type = V4L2_INPUT_TYPE_CAMERA;
        if (copy_to_user(u, &inp, sizeof(inp)))
            return -EFAULT;
        return 0;
    }
    case 39:                        /* VIDIOC_S_INPUT */
        return arg ? -EINVAL : 0;
    case 38: {                      /* VIDIOC_G_INPUT — camera 0 */
        int idx = 0;
        if (copy_to_user(u, &idx, sizeof(idx)))
            return -EFAULT;
        return 0;
    }
    case 23: {                      /* VIDIOC_G_STD — no standard */
        v4l2_std_id id = 0;
        if (copy_to_user(u, &id, sizeof(id)))
            return -EFAULT;
        return 0;
    }
    case 63: {                      /* VIDIOC_QUERYSTD */
        v4l2_std_id id = 0;
        if (copy_to_user(u, &id, sizeof(id)))
            return -EFAULT;
        return 0;
    }
    case 25:                        /* VIDIOC_ENUMSTD — none */
    case 37:                        /* VIDIOC_QUERYMENU — none */
    case 27:                        /* VIDIOC_G_CTRL — none */
    case 28:                        /* VIDIOC_S_CTRL — none */
    case 71:                        /* VIDIOC_G_EXT_CTRLS */
    case 72:                        /* VIDIOC_S_EXT_CTRLS */
        return -EINVAL;
    case 58: {                      /* VIDIOC_CROPCAP */
        struct v4l2_cropcap cc;
        if (copy_from_user(&cc, u, sizeof(cc)))
            return -EFAULT;
        if (cc.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
            return -EINVAL;
        memset(&cc, 0, sizeof(cc));
        cc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        cc.bounds.left = 0;
        cc.bounds.top = 0;
        cc.bounds.width  = cam_fmt.width;
        cc.bounds.height = cam_fmt.height;
        cc.defrect = cc.bounds;
        if (copy_to_user(u, &cc, sizeof(cc)))
            return -EFAULT;
        return 0;
    }
    case 59: {                      /* VIDIOC_G_CROP */
        struct v4l2_crop c;
        if (copy_from_user(&c, u, sizeof(c)))
            return -EFAULT;
        if (c.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
            return -EINVAL;
        c.c.left = 0;
        c.c.top = 0;
        c.c.width  = cam_fmt.width;
        c.c.height = cam_fmt.height;
        if (copy_to_user(u, &c, sizeof(c)))
            return -EFAULT;
        return 0;
    }
    case 60:                        /* VIDIOC_S_CROP — accept any */
        return 0;
    case 94: {                      /* VIDIOC_G_SELECTION */
        struct v4l2_selection sel;
        if (copy_from_user(&sel, u, sizeof(sel)))
            return -EFAULT;
        if (sel.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
            return -EINVAL;
        sel.r.left = 0;
        sel.r.top = 0;
        sel.r.width  = cam_fmt.width;
        sel.r.height = cam_fmt.height;
        if (copy_to_user(u, &sel, sizeof(sel)))
            return -EFAULT;
        return 0;
    }
    case 95:                        /* VIDIOC_S_SELECTION — accept any */
        return 0;
    case 36: {                      /* VIDIOC_QUERYCTRL — none offered */
        struct v4l2_queryctrl qc;
        if (copy_from_user(&qc, u, sizeof(qc)))
            return -EFAULT;
        rc = -EINVAL;
        break;
    }
    default:
        return -ENOIOCTLCMD;
    }
    return rc;
}

static const struct file_operations cam_fops = {
    .owner          = THIS_MODULE,
    .open           = cam_open,
    .release        = cam_release,
    .unlocked_ioctl = cam_ioctl,
    .compat_ioctl   = cam_ioctl,
    .poll           = cam_poll,
    .mmap           = cam_mmap,
};

static dev_t cam_devno;
static struct cdev cam_cdev;
static struct class *cam_class;

/* ------------------------- producer /dev/c920feed --------------------- */
static ssize_t feed_write(struct file *f, const char __user *buf,
                          size_t count, loff_t *ppos)
{
    size_t want;
    mutex_lock(&cam_lock);
    want = cam_fmt.sizeimage;
    if (count != want || !frame_buf) {
        mutex_unlock(&cam_lock);
        return -EINVAL;
    }
    if (copy_from_user(frame_buf, buf, want)) {
        mutex_unlock(&cam_lock);
        return -EFAULT;
    }
    frame_seq++;
    have_frame = 1;
    mutex_unlock(&cam_lock);
    wake_up_interruptible(&frame_wait);
    return want;
}

static const struct file_operations feed_fops = {
    .owner = THIS_MODULE,
    .open  = generic_file_open,
    .write = feed_write,
};

static struct miscdevice feed_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = FEED_NAME,             /* /dev/c920feed */
    .fops  = &feed_fops,
};

/* -------------------------------- init -------------------------------- */
static int __init cam_init(void)
{
    int rc;

    init_waitqueue_head(&frame_wait);
    frame_buf = vmalloc_user(cam_fmt.sizeimage);
    if (!frame_buf)
        return -ENOMEM;

    rc = alloc_chrdev_region(&cam_devno, 0, 1, DRV_NAME);
    if (rc)
        goto err_buf;
    cdev_init(&cam_cdev, &cam_fops);
    cam_cdev.owner = THIS_MODULE;
    rc = cdev_add(&cam_cdev, cam_devno, 1);
    if (rc)
        goto err_region;
    cam_class = class_create(DRV_NAME);
    if (IS_ERR(cam_class)) {
        rc = PTR_ERR(cam_class);
        goto err_cdev;
    }
    device_create(cam_class, NULL, cam_devno, NULL, "video%d", CAM_NODE);

    rc = misc_register(&feed_dev);
    if (rc)
        goto err_class;

    pr_info(DRV_NAME ": /dev/video%d (BGR3) + /dev/%s — ours\n",
            CAM_NODE, FEED_NAME);
    return 0;

err_class:
    class_destroy(cam_class);
err_cdev:
    cdev_del(&cam_cdev);
err_region:
    unregister_chrdev_region(cam_devno, 1);
err_buf:
    vfree(frame_buf);
    return rc;
}

static void __exit cam_exit(void)
{
    misc_deregister(&feed_dev);
    device_destroy(cam_class, cam_devno);
    class_destroy(cam_class);
    cdev_del(&cam_cdev);
    unregister_chrdev_region(cam_devno, 1);
    vfree(frame_buf);
    pr_info(DRV_NAME ": unloaded\n");
}

module_init(cam_init);
module_exit(cam_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Natalie — AcreetionOS");
MODULE_DESCRIPTION("Our own BGR3 virtual camera (no v4l2 framework)");
