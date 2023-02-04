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
  
## Received Message Pool

-  Received messages are stored in a [ThreadX block pool](https://learn.microsoft.com/en-us/azure/rtos/threadx/appendix-a#block-memory-services).
- This allows space for a message to be dynamically allocated in the CAN ISR 
  (this is not possible with byte pools), however the message size allocated
  has to be large enough for the biggest possible CAN message when working
  with block pools.
- Received messages stored in the block pool are distributed to all subscribers
  of that CAN ID. Subscribers receive a **pointer** to the message in the pool
  which prevents unnecessary copying of the message. This does mean that messages
  should be treated as read-only by subscribers, but a copy can be made in the
  rare cases where a subscriber wants to modify the message.
- Each message has a ["reference count"](https://en.wikipedia.org/wiki/Reference_counting)
  which is initially equal to the number of subscribers to that message. When
  a subscriber has finished with the received message, it must call the
  `rtcan_consumed()` function to indicate this and decrement the reference
  count. When the reference count reaches zero, the message can be deleted from
  the block pool to free the memory.
- Note that ideally, subscribers should consume messages as quickly as possible
  to free up the block again for other messages. The size of the block pool
  should have enough overhead that it doesn't repeatedly run out of memory.
  The specific behaviour and requirements here will depend on the application's
  use of RTCAN and received messages.

## Tradeoffs / Limitations / Other Options

- For small numbers of subscribers, a simple predetermined look-up table would
  be far simpler to implement and more memory efficient, however this does not
  scale well. In such a scenario, you can simply not use this feature of RTCAN.
- Hash maps require a significant amount of memory and necessitate dynamic
  memory allocation. The use of ThreadX memory pool services ensures that 
  dynamic allocations are deterministic, however it is recommended to set up all 
  subscriptions on initialisation if possible.
