#include <pthread.h>
#include "queue.h"
using namespace wd;
Queue::Queue(int capacity)
{
    this->capacity=capacity;
    front=rear=0;
    size=0;
    buf=new  char [capacity];
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&nfull_cond, NULL);
    pthread_cond_init(&nempty_cond, NULL);
    
}
Queue::~Queue()
{
    if(buf)
    {
        delete buf;
        buf=nullptr;
    }
    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&nfull_cond);
    pthread_cond_destroy(&nempty_cond);
}
bool Queue::is_empty()
{
    return  size==0;
}
int Queue::get_size()
{
    return  size;
}
bool Queue::get_first(char &data)
{
       pthread_mutex_lock(&mutex);
      if(is_empty())
      {
         pthread_mutex_unlock(&mutex);  
           return false;
      }
        data= buf[rear];
        pthread_mutex_unlock(&mutex);  
      return true;
}
bool Queue::get_second(char &data)
{
       pthread_mutex_lock(&mutex);
      if(size<2)
      {
         pthread_mutex_unlock(&mutex);  
           return false;
      }
        data= buf[(rear+1)%capacity];
        pthread_mutex_unlock(&mutex);  
      return true;
}
bool Queue::enqueue(char data )
{
    pthread_mutex_lock(&mutex);
    if(size==capacity)
    {
           pthread_mutex_unlock(&mutex);  
           return false;
    }
     buf[front++]=data;
    front=front%capacity;
      size++;
            pthread_cond_signal(&nempty_cond);
    pthread_mutex_unlock(&mutex);  
    return true;    
}
int Queue::enqueue(char* buff,int len )
{
    if(!len)
        return 0;
    pthread_mutex_lock(&mutex);
      while(size==capacity)
     pthread_cond_wait(&nfull_cond,&mutex);
      int len1=capacity-size;
      len1=len1<len?len1:len;
    int i=0;
    while(i<len1)
    {  
          buf[front++]=buff[i++];
        front=front%capacity;
           size++;
        
    }
    pthread_cond_signal(&nempty_cond);
    pthread_mutex_unlock(&mutex);  
    return len1;
}
bool Queue::dequeue(char& data)
{
    pthread_mutex_lock(&mutex); 
    if(size==0)
    {
         pthread_mutex_unlock(&mutex);  
           return false;
    }
     size--;
     data= buf[rear++];
     rear=rear%capacity;
    pthread_cond_signal(&nfull_cond);
 pthread_mutex_unlock(&mutex);  
    return true;
}
int Queue::dequeue(char* buff,int len )
{
    if(!len)
    return 0;
    pthread_mutex_lock(&mutex); 
      while(size==0)
      pthread_cond_wait(&nempty_cond,&mutex);
    int len1;
    len1=size<len?size:len;
    int i=0;
    while(i<len1)
    {
         buff[i++]= buf[rear++];
          rear=rear%capacity;
          size--;
        
    }
    pthread_cond_signal(&nfull_cond);
    pthread_mutex_unlock(&mutex);  
    return len1;
}
