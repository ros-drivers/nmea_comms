^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nmea_comms
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2013-10-03)
------------------
* Add sleep() call between poll and read to further reduce CPU use.
* Properly apply the specified baud rate.
* Gracefully handle nulls received on the serial line.

0.0.2 (2013-09-23)
------------------
* Add support for setting frame_id on rx Sentence messages.

0.0.1 (2013-09-03)
------------------
* Initial release of bidirectional socket and serial nodes.
