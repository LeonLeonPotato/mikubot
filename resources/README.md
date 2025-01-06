# Resources

This directory is a PROS template containing cold resources to be used in the main Mikubot project. This includes things like fonts, GIFs, images, etc. that are too big to be added as c / header files to the hot executable, as this would slow down upload times.

The solution to this is to store these big blocks of data into a separate library which could then be uploaded as a cold asset, preventing reuploading them every time and speeding up the upload process.
