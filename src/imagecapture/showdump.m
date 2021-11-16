% Read dump from imageCapture.c and display - for debugging

f=fopen('satdump.dat');
im=fread(f,[768 1024],'double');
fclose(f);
im=im';
figure(1);clf;imagesc(im);axis image;colormap(gray);\
