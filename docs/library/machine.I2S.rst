.. currentmodule:: machine
.. _machine.I2S:

class I2S -- a serial protocol for connecting digital audio devices
===================================================================

I2S (Inter-IC Sound), is an electrical serial bus interface standard used for connecting digital audio devices together. 
At the physical level it consists of 3 wires: SCK (serial clock), WS (word select) and SD (serial data).  
An optional 4th MCK (master clock) wire may be present.

I2S objects are created attached to a specific peripheral.  They can be initialised
when created, or initialised later on.

Printing the I2S object gives you information about its configuration.

Example usage::

    <coming soon>

Constructors
------------

.. class:: I2S(id, sck=None, ws=None, sd=None, mode=None, bits=32, format=None, rate=None)

   Construct an I2S object of the given id.
   
   - ``id`` identifies a particular I2S peripheral.  Values of ``id`` depend on a particular port and its hardware. Values 0, 1, etc. are commonly used.
   - ``sck`` is a pin object for the serial clock line
   - ``ws`` is a pin object for the word select line
   - ``sd`` is a pin object for the serial data line
   - ``mode`` specifies receive or transmit
   - ``bits`` specifies number of bits in each Word transmitted on a channel
   - ``format`` specifies channel format
   - ``rate`` specifies audio sampling rate (samples/s)

Methods
-------

.. method:: I2S.init(sck, ...)

  see Constructor for argument descriptions
     
.. method:: I2S.deinit()

  Deinitialize the I2S peripheral
  
.. method::  I2S.readinto(buf, handler)

  Read audio samples into the buffer specified by ``buf``.  ``buf`` must support the buffer protocol, such as bytearray or array.
  ``handler`` is a callback indicating ``buf`` is filled.  If ``handler`` is specified the method is non-blocking, 
  otherwise the method blocks until ``buf`` is filled.  
  Returns number of bytes read 
  
.. method::  I2S.write(buf, handler)

  Write audio samples contained in ``buf``. ``buf`` must support the buffer protocol, such as bytearray or array.
  ``handler`` is a callback indicating ``buf`` is emptied.  If ``handler`` is specified the method is non-blocking, 
  otherwise the method blocks until ``buf`` is emptied.  
  Returns number of bytes written 
  
Constants
---------

.. data:: I2S.RX

   for initialising the I2S peripheral ``mode`` to receive

.. data:: I2S.TX

   for initialising the I2S peripheral ``mode`` to transmit

.. data:: I2S.STEREO

   for initialising the I2S peripheral ``format`` to stereo

.. data:: I2S.MONO_RIGHT

   for initialising the I2S peripheral ``format`` to mono right channel

.. data:: I2S.MONO_LEFT

   for initialising the I2S peripheral ``format`` to mono left channel

Implementation-specific details
-------------------------------

Different implementations of the ``I2S`` class on different hardware support
varying subsets of the options above.

PyBoard
```````
- Pyboard v1.0/v1.1 supports 1 I2S peripheral
- Pyboard D-series support 2 I2S peripheral

ESP32
`````
Supports 2 I2S peripherals