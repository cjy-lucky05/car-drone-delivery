#ifndef __QUEUUE_H__
#define __QUEUUE_H__
#include <pthread.h>
namespace wd
{
class  Queue
{
public:
        Queue(int capacity);
        ~Queue();
        int  enqueue(char* buf,int len );
        int  dequeue(char* buf,int len );
        bool enqueue(char data );
        bool dequeue(char  &data );
        bool is_empty();
        bool get_first(char &data);
        bool get_second(char &data);
        int get_size();
private:
        char*buf;
        int  capacity;
        int front;
        int  rear;
        int size;
       pthread_mutex_t  mutex;
       pthread_cond_t   nfull_cond;
     pthread_cond_t   nempty_cond;
};
#endif // !
}





