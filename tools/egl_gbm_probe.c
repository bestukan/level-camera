#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <gbm.h>

#ifndef EGL_PLATFORM_GBM_KHR
#define EGL_PLATFORM_GBM_KHR 0x31D7
#endif

#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif

typedef EGLDisplay (*PFNEGLGETPLATFORMDISPLAYEXTPROC_LOCAL)(EGLenum platform,
                                                            void *native_display,
                                                            const EGLint *attrib_list);

static int has_extension(const char *extensions, const char *name)
{
    const char *p;
    size_t name_len;

    if (!extensions || !name)
        return 0;

    name_len = strlen(name);
    p = extensions;
    while ((p = strstr(p, name)) != NULL) {
        int before_ok = (p == extensions) || (p[-1] == ' ');
        int after_ok = (p[name_len] == '\0') || (p[name_len] == ' ');

        if (before_ok && after_ok)
            return 1;

        p += name_len;
    }

    return 0;
}

static const char *safe_string(const char *s)
{
    return s ? s : "(null)";
}

static EGLDisplay get_gbm_egl_display(struct gbm_device *gbm)
{
    PFNEGLGETPLATFORMDISPLAYEXTPROC_LOCAL get_platform_display;
    const char *client_ext;

    client_ext = eglQueryString(EGL_NO_DISPLAY, EGL_EXTENSIONS);
    printf("EGL client extensions:\n%s\n\n", safe_string(client_ext));

    get_platform_display =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC_LOCAL)eglGetProcAddress("eglGetPlatformDisplayEXT");
    if (get_platform_display &&
        (has_extension(client_ext, "EGL_EXT_platform_base") ||
         has_extension(client_ext, "EGL_KHR_platform_base") ||
         has_extension(client_ext, "EGL_KHR_platform_gbm") ||
         has_extension(client_ext, "EGL_MESA_platform_gbm"))) {
        EGLDisplay display;

        display = get_platform_display(EGL_PLATFORM_GBM_KHR, gbm, NULL);
        if (display != EGL_NO_DISPLAY)
            return display;
    }

    printf("warning: eglGetPlatformDisplayEXT(GBM) unavailable, trying eglGetDisplay(gbm)\n");
    return eglGetDisplay((EGLNativeDisplayType)gbm);
}

static int choose_config(EGLDisplay display, EGLConfig *config)
{
    static const EGLint attrs[] = {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 0,
        EGL_NONE
    };
    EGLint count = 0;

    if (!eglChooseConfig(display, attrs, config, 1, &count) || count <= 0)
        return -1;

    return 0;
}

static struct gbm_surface *create_test_surface(struct gbm_device *gbm)
{
    struct gbm_surface *surface;

    surface = gbm_surface_create(gbm,
                                 64,
                                 64,
                                 GBM_FORMAT_XRGB8888,
                                 GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
    if (surface)
        return surface;

    printf("warning: gbm_surface_create(scanout|rendering) failed: %s\n",
           strerror(errno));

    surface = gbm_surface_create(gbm,
                                 64,
                                 64,
                                 GBM_FORMAT_XRGB8888,
                                 GBM_BO_USE_RENDERING);
    if (surface)
        return surface;

    printf("warning: gbm_surface_create(rendering) failed: %s\n",
           strerror(errno));
    return NULL;
}

int main(int argc, char **argv)
{
    const char *drm_dev = "/dev/dri/card0";
    int drm_fd = -1;
    struct gbm_device *gbm = NULL;
    struct gbm_surface *gbm_surface = NULL;
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLConfig config = NULL;
    EGLSurface egl_surface = EGL_NO_SURFACE;
    EGLContext context = EGL_NO_CONTEXT;
    EGLint major = 0;
    EGLint minor = 0;
    const char *egl_ext;
    const char *gl_ext;
    static const EGLint context_attrs[] = {
        EGL_CONTEXT_CLIENT_VERSION, 2,
        EGL_NONE
    };

    if (argc > 1)
        drm_dev = argv[1];

    printf("DRM device: %s\n", drm_dev);

    drm_fd = open(drm_dev, O_RDWR | O_CLOEXEC);
    if (drm_fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", drm_dev, strerror(errno));
        return 1;
    }

    gbm = gbm_create_device(drm_fd);
    if (!gbm) {
        fprintf(stderr, "gbm_create_device failed\n");
        close(drm_fd);
        return 1;
    }
    printf("GBM backend: %s\n", safe_string(gbm_device_get_backend_name(gbm)));

    display = get_gbm_egl_display(gbm);
    if (display == EGL_NO_DISPLAY) {
        fprintf(stderr, "get EGLDisplay from GBM failed\n");
        goto fail;
    }

    if (!eglInitialize(display, &major, &minor)) {
        fprintf(stderr, "eglInitialize failed, error=0x%x\n", eglGetError());
        goto fail;
    }

    printf("EGL initialized: %d.%d\n", major, minor);
    printf("EGL vendor: %s\n", safe_string(eglQueryString(display, EGL_VENDOR)));
    printf("EGL version: %s\n", safe_string(eglQueryString(display, EGL_VERSION)));
    printf("EGL APIs: %s\n", safe_string(eglQueryString(display, EGL_CLIENT_APIS)));

    egl_ext = eglQueryString(display, EGL_EXTENSIONS);
    printf("EGL display extensions:\n%s\n\n", safe_string(egl_ext));

    printf("EGL_KHR_platform_gbm: %s\n",
           has_extension(egl_ext, "EGL_KHR_platform_gbm") ? "yes" : "no");
    printf("EGL_MESA_platform_gbm: %s\n",
           has_extension(egl_ext, "EGL_MESA_platform_gbm") ? "yes" : "no");
    printf("EGL_KHR_image_base: %s\n",
           has_extension(egl_ext, "EGL_KHR_image_base") ? "yes" : "no");
    printf("EGL_EXT_image_dma_buf_import: %s\n",
           has_extension(egl_ext, "EGL_EXT_image_dma_buf_import") ? "yes" : "no");
    printf("EGL_EXT_image_dma_buf_import_modifiers: %s\n",
           has_extension(egl_ext, "EGL_EXT_image_dma_buf_import_modifiers") ? "yes" : "no");

    if (!eglBindAPI(EGL_OPENGL_ES_API)) {
        fprintf(stderr, "eglBindAPI(GLES) failed, error=0x%x\n", eglGetError());
        goto fail;
    }

    if (choose_config(display, &config) < 0) {
        fprintf(stderr, "eglChooseConfig failed, error=0x%x\n", eglGetError());
        goto fail;
    }

    gbm_surface = create_test_surface(gbm);
    if (!gbm_surface)
        goto fail;

    egl_surface = eglCreateWindowSurface(display,
                                         config,
                                         (EGLNativeWindowType)gbm_surface,
                                         NULL);
    if (egl_surface == EGL_NO_SURFACE) {
        fprintf(stderr, "eglCreateWindowSurface failed, error=0x%x\n", eglGetError());
        goto fail;
    }

    context = eglCreateContext(display, config, EGL_NO_CONTEXT, context_attrs);
    if (context == EGL_NO_CONTEXT) {
        fprintf(stderr, "eglCreateContext(GLES2) failed, error=0x%x\n", eglGetError());
        goto fail;
    }

    if (!eglMakeCurrent(display, egl_surface, egl_surface, context)) {
        fprintf(stderr, "eglMakeCurrent failed, error=0x%x\n", eglGetError());
        goto fail;
    }

    printf("\nGL vendor: %s\n", safe_string((const char *)glGetString(GL_VENDOR)));
    printf("GL renderer: %s\n", safe_string((const char *)glGetString(GL_RENDERER)));
    printf("GL version: %s\n", safe_string((const char *)glGetString(GL_VERSION)));

    gl_ext = (const char *)glGetString(GL_EXTENSIONS);
    printf("GL extensions:\n%s\n\n", safe_string(gl_ext));
    printf("GL_OES_EGL_image: %s\n",
           has_extension(gl_ext, "GL_OES_EGL_image") ? "yes" : "no");
    printf("GL_OES_EGL_image_external: %s\n",
           has_extension(gl_ext, "GL_OES_EGL_image_external") ? "yes" : "no");

    if (has_extension(egl_ext, "EGL_EXT_image_dma_buf_import") &&
        has_extension(gl_ext, "GL_OES_EGL_image")) {
        printf("\nprobe result: OK, GBM + EGL + GLES2 are usable, "
               "dmabuf -> EGLImage is likely supported.\n");
    } else {
        printf("\nprobe result: GBM + EGL + GLES2 initialized, but dmabuf/EGLImage "
               "support is incomplete. Check the extension lines above.\n");
    }

    eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(display, context);
    eglDestroySurface(display, egl_surface);
    eglTerminate(display);
    gbm_surface_destroy(gbm_surface);
    gbm_device_destroy(gbm);
    close(drm_fd);
    return 0;

fail:
    if (display != EGL_NO_DISPLAY) {
        eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (context != EGL_NO_CONTEXT)
            eglDestroyContext(display, context);
        if (egl_surface != EGL_NO_SURFACE)
            eglDestroySurface(display, egl_surface);
        eglTerminate(display);
    }
    if (gbm_surface)
        gbm_surface_destroy(gbm_surface);
    if (gbm)
        gbm_device_destroy(gbm);
    if (drm_fd >= 0)
        close(drm_fd);
    return 1;
}
