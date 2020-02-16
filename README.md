# Paper Tape Reader

This is a DIY reader for 8-bit (9-hole) vintage paper tapes.

![FullPicture](/images/FullPicture.jpg)

For a short demonstration see my video here:
https://www.youtube.com/watch?v=uZ2VWKNlPQU

## Building the reader head

When I started with this project, constructing the reader head was
the most uncertain aspect. I found some other DIY attempts on the web
but none seemed ideal. I wanted to use miniature photo-transistors that
could fit on a 0.1" grid (to match the .1" spacing of the paper tape holes).
I found the [Kingbright AM2520P3BT03](https://www.digikey.com/product-detail/en/kingbright/AM2520P3BT03/754-2338-1-ND/9647017)
IR phototransistors and wavelength-matched 
[ON Semiconductor QEB363ZR](https://www.digikey.com/product-detail/en/on-semiconductor/QEB363ZR/QEB363ZRCT-ND/3479521)
IR LEDs. Initial tests with these mounted on a perfboard were very promising,
so I made two matched PCBs ([emitter](schematics/readerhead-emitter-pcb.pdf) 
and [sensor](schematics/readerhead-sensor-pcb.pdf)) that can be sandwiched together
to form the reader head:

![ReaderHeadTop](/images/ReaderHeadTop.jpg | width=200) ![ReaderHeadBottom](/images/ReaderHeadTop.jpg | width=200)
