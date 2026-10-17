#include "BufferQueue.h"
#include <mutex>
#include <condition_variable>
#include <list>
#include <memory>
#include <iostream>
#include <chrono>

struct BufferQueue {
    std::mutex freeMutex;
    std::mutex readyMutex;
    std::condition_variable freeWait;
    std::condition_variable readyWait;
    std::list<DataQueMsg*> freeBuffers;
    std::list<DataQueMsg*> readyBuffers;
    bool isCircular; // 是否为环形缓冲区
    int maxBuffers;  // 最大缓冲区数量
};

BufferQueueHandle CreateBufferQueue(int nBuf, int szNode, bool circular) {
    if (nBuf <= 0 || szNode <= 0) {
        std::cerr << "Invalid parameters: nBuf = " << nBuf << ", szNode = " << szNode << std::endl;
        return nullptr;
    }

    std::unique_ptr<BufferQueue> queue(new (std::nothrow) BufferQueue);
    if (!queue) {
        std::cerr << "Failed to allocate memory for BufferQueue" << std::endl;
        return nullptr;
    }

    queue->isCircular = circular;
    queue->maxBuffers = nBuf;

    for (int i = 0; i < nBuf; ++i) {
        DataQueMsg* buffer = static_cast<DataQueMsg*>(malloc(szNode + sizeof(DataQueMsg)));
        if (!buffer) {
            std::cerr << "Failed to allocate memory for buffer " << i << std::endl;
            return nullptr;
        }
        buffer->szBuf = szNode;
        buffer->szData = 0;
        queue->freeBuffers.push_back(buffer);
    }

    return queue.release();
}

void DestroyBufferQueue(BufferQueueHandle handle) {
    if (!handle) {
        std::cerr << "Null buffer queue handle" << std::endl;
        return;
    }

    BufferQueue* queue = handle;

    {
        std::lock_guard<std::mutex> locker(queue->freeMutex);
        for (DataQueMsg* buffer : queue->freeBuffers) {
            free(buffer);
        }
        queue->freeBuffers.clear();
        queue->freeWait.notify_all();
    }
    {
        std::lock_guard<std::mutex> locker(queue->readyMutex);
        for (DataQueMsg* buffer : queue->readyBuffers) {
            free(buffer);
        }
        queue->readyBuffers.clear();
        queue->readyWait.notify_all();
    }

    delete queue;
}

DataQueMsg* GetFreeBuffer(BufferQueueHandle handle, int timeoutMs) {
    if (!handle) {
        std::cerr << "Null buffer queue handle" << std::endl;
        return nullptr;
    }

    BufferQueue* queue = handle;
    std::unique_lock<std::mutex> locker(queue->freeMutex);

    if (queue->freeBuffers.empty()) {
        if (!queue->freeWait.wait_for(locker, std::chrono::milliseconds(timeoutMs), 
            [&]() { return !queue->freeBuffers.empty(); })) {
            return nullptr;
        }
    }

    if (queue->freeBuffers.empty()) {
        return nullptr;
    }

    DataQueMsg* buffer = queue->freeBuffers.front();
    queue->freeBuffers.pop_front();
    return buffer;
}

void PutFreeBuffer(BufferQueueHandle handle, DataQueMsg* buffer) {
    if (!handle || !buffer) {
        std::cerr << "Invalid parameters: handle = " << handle << ", buffer = " << buffer << std::endl;
        return;
    }

    BufferQueue* queue = handle;
    std::lock_guard<std::mutex> locker(queue->freeMutex);
    buffer->szData = 0;
    queue->freeBuffers.push_back(buffer);
    queue->freeWait.notify_one();
}

DataQueMsg* GetReadyBuffer(BufferQueueHandle handle, int timeoutMs) {
    if (!handle) {
        std::cerr << "Null buffer queue handle" << std::endl;
        return nullptr;
    }

    BufferQueue* queue = handle;
    std::unique_lock<std::mutex> locker(queue->readyMutex);

    if (queue->readyBuffers.empty()) {
        if (!queue->readyWait.wait_for(locker, std::chrono::milliseconds(timeoutMs), 
            [&]() { return !queue->readyBuffers.empty(); })) {
            return nullptr;
        }
    }

    if (queue->readyBuffers.empty()) {
        return nullptr;
    }

    DataQueMsg* buffer = queue->readyBuffers.front();
    queue->readyBuffers.pop_front();
    return buffer;
}

void PutReadyBuffer(BufferQueueHandle handle, DataQueMsg* buffer) {
    if (!handle || !buffer) {
        std::cerr << "Invalid parameters: handle = " << handle << ", buffer = " << buffer << std::endl;
        return;
    }

    BufferQueue* queue = handle;
    std::unique_lock<std::mutex> locker(queue->readyMutex);

    if (queue->isCircular && queue->readyBuffers.size() >= static_cast<size_t>(queue->maxBuffers)) {
        DataQueMsg* oldBuffer = queue->readyBuffers.front();
        queue->readyBuffers.pop_front();
        std::lock_guard<std::mutex> freeLocker(queue->freeMutex);
        oldBuffer->szData = 0;
        queue->freeBuffers.push_back(oldBuffer);
        queue->freeWait.notify_one();
    }

    queue->readyBuffers.push_back(buffer);
    queue->readyWait.notify_one();
}