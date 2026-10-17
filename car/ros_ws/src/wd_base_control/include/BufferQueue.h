#ifndef BUFQUE_H
#define BUFQUE_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 数据消息结构
typedef struct {
    uint32_t szBuf;  // 缓冲区大小
    uint32_t szData; // 数据大小
    char buff[0];    // 柔性数组，用于存储数据
} DataQueMsg;

// 缓冲队列句柄
struct BufferQueue;
typedef BufferQueue* BufferQueueHandle;

// 创建和销毁缓冲队列
BufferQueueHandle CreateBufferQueue(int nBuf, int szNode, bool circular);
void DestroyBufferQueue(BufferQueueHandle handle);

// 缓冲队列操作
DataQueMsg* GetFreeBuffer(BufferQueueHandle handle, int timeoutMs);
void PutFreeBuffer(BufferQueueHandle handle, DataQueMsg* buffer);

DataQueMsg* GetReadyBuffer(BufferQueueHandle handle, int timeoutMs);
void PutReadyBuffer(BufferQueueHandle handle, DataQueMsg* buffer);

#ifdef __cplusplus
}
#endif

#endif // BUFQUE_H