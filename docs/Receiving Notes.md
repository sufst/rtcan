# RTCAN Receiving Functionality

This document provides some notes on the receiving functionality of RTCAN for
future developers.

## Design Principle

The high level design is based around the "publisher/subscriber" pattern in
which:

- "Publishers" are instances of the RTCAN service.
- "Subscribers" are any thread which wants to receive a CAN message with a 
  particular ID.
- Communication is asynchronous, with the "broker" (or "event channel") being a
  ThreadX queue provided by subscribers when they subscribe to a CAN message.

## Listener List Implementation

Each RTCAN maintains a list of subscribers to which CAN messages are dispatched.
This was implemented as follows:

- The list of subscribers is stored as entries in a hash table to allow for fast
  look-up.
- The hash is computed from the value of the CAN ID. 
- The index of a given CAN ID's entry in the table is given by the hash modulo
  the table size.
- Each table entry consists of a singly linked list of subscribers to the 
  corresponding CAN message ID. Each subscriber provides a queue as a
  destination for incoming messages which is stored in its corresponding list 
  item.
- In the event of a collision (two CAN IDs mapping to the same table entry), 
  [separate chaining](https://en.wikipedia.org/wiki/Hash_table#Separate_chaining) 
  is used. Entries in the table therefore also include a pointer to chained
  items for that table entry.
- All data which is added to the table is dynamically allocated from a byte
  pool. The size of this pool limits the maximum number of subscriptions which
  can be made.


## Tradeoffs / Limitations / Other Options

- For small numbers of subscribers, a simple predetermined look-up table would
  be far simpler to implement and more memory efficient, however this does not
  scale well. In such a scenario, you can simply not use this feature of RTCAN.
- Hash maps require a significant amount of memory and necessitate dynamic
  memory allocation. The use of ThreadX memory pool services ensures that 
  dynamic allocations are deterministic, however it is recommended to set up all 
  subscriptions on initialisation if possible.