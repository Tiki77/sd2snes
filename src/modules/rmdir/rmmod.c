/* rmdir.bin: the file manager's "Delete folder" (FM_OP_RMDIR), on its own image.  It is
   ../fileops/fopmod.c built with only the delete in it: fileops.bin is right under its
   memory ceiling (the image and its spare RAM share the firmware's module block), so the
   delete could not join it.  See fopmod.c. */
#define FOP_RMDIR_ONLY
#include "../fileops/fopmod.c"
