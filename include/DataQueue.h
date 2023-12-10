#ifndef DATA_QUEUE_H
#define DATA_QUEUE_H

#include "data_structure.h"
#include <queue>

// Properties of Data Queue class
// Create functions to manage a queue to ensure proper handling of the data. 
// Functions
// 1. enqueue
// 2. dequeue
class DataQueue
{
    public:
        // Default constructor
        DataQueue()
        {

        }

        // Default Destructor
        ~DataQueue()
        {

        }

        // Default copy constructor
        DataQueue(const DataQueue &dataQueue)
        {

        }

        // Copy assignment operator
        DataQueue &operator=(const DataQueue &dataQueue)
        {

        }

        // Default move constructor
        DataQueue(DataQueue &&dataQueue)
        {

        }

        // Move assignment operator
        DataQueue &operator=(DataQueue &&dataQueue)
        {

        }

        // Member functions

        // Comparison operator
        bool &operator==(DataQueue &dataQueue)
        {
            bool returnValue = false;

            return returnValue;
        }

        //DataStruct &dataStructure;
        std::priority_queue<DataStruct, vector<DataStruct>> dataQueue;
        void enqueue(DataStruct &dataStructure);
        void deque();
    private:
        DataQueue dataQueueObject;
};


#endif