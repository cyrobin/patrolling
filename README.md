patrolling
==========

Python Patrolling Planner (using GLPK)

Maps
****

Maps file in use are geotiff (either .tif or .png + .aux.xml)
To resize/modify, use ``gdalwrap`` (for GTif) or ``gdal_translate`` (for png) :

.. code_block:: gdal_translate -outsize xsize ysize -of PNG map.png resized_map.png

To get info, use :

.. code_block:: gdalinfo map.png

License
*******

Under BSD-3-clauses licence.
2014 - Cyril Robin -- LAAS / CNRS




