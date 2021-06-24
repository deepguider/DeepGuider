#ifndef __DG_UTILS_UTILITY__
#define __DG_UTILS_UTILITY__

#include <vector>
#include <string>
#include <assert.h>

namespace dg
{

    /**
     * @brief FIFO(First In First Out) ������ ����
     *
     * ���� �ֱ��� n���� �����͸��� �����ϱ� ���� �ڷᱸ�� (���ø� ����)
     */
    template <class T>
    class RingBuffer
    {
    public:
        /**
        * �⺻ ������ (empty ������ ����)
        */
        RingBuffer();

        /**
        * ������
        * @param size ������ �ִ� ũ�� (������ �������� ����)
        */
        RingBuffer(int size);

        /** �Ҹ��� */
        virtual ~RingBuffer();

        /**
        * �����͸� ������ ���� ���� (���� �ִ�ũ�⿡ ������ ��� ���� ������ �����͸� ����)
        * @param data �߰��� ������
        */
        virtual void push_back(const T& data);

        /**
        * �������� ������ ����(���� ����� ������ ����)�� ��ȯ
        * @return ������ ������ ����
        */
        int data_count() const;

        /**
        * �������� ũ��(������ �ִ� ������ ����)�� ��ȯ
        * @return ������ ũ��
        */
        int size() const;

        /**
        * �������� ũ�⸦ �Է� size�� ����
        * @param size ������ �ִ� ũ�� (������ �������� ����)
        */
        void resize(int size);

        /**
        * ������ ��ø (���� ������ ������ i��° ������ ��ȯ)
        * @return �����ۿ� ����� �ֱ� i��° ������
        */
        T& operator [](int i);

        /**
        * ������ ��ø (���� ������ ������ i��° ������ ��ȯ)
        * @return �����ۿ� ����� �ֱ� i��° ������
        */
        const T operator [](int i) const;

    protected:
        /** ������ �޸� */
        T* m_buf;

        /** ������ �ִ� ũ�� */
        int m_buf_size;

        /** ������ ����� �������� ��ġ(���� �ε���) */
        int m_next_index;

        /** ���� ����� ������ ���� */
        int m_data_count;
    };

    /**
    * @brief �����Ͱ� ������ ������ ����
    *
    * sum(), average() ���� ���� �����Ϳ� Ưȭ�� �߰� ���� ����
    */
    template <class T>
    class RingBufferNumeric : public RingBuffer<T>
    {
    public:
        /**
        * �⺻ ������ (empty ������ ����)
        */
        RingBufferNumeric();

        /**
        * ������
        * @param size ������ �ִ� ũ�� (������ �������� ����)
        */
        RingBufferNumeric(int size);

        /**
        * �����͸� ������ ���� ���� (���� �ִ�ũ�⿡ ������ ��� ���� ������ �����͸� ����)
        * @param data �߰��� ������
        */
        void push_back(const T& data);

        /**
        * ���� ���ۿ� ����� ������ ���� ��ȯ
        * @return ������ ��
        */
        T sum();

        /**
        * ���� ���ۿ� ����� ������ ����� ��ȯ
        * @return ������ ���
        */
        double average();

    protected:
        /** �����ۿ� ����� ������ �� */
        T m_sum;
    };

    template<class T>
    RingBuffer<T>::RingBuffer()
    {
        m_buf = NULL;
        m_buf_size = 0;
        m_next_index = 0;
        m_data_count = 0;
    }

    template<class T>
    RingBuffer<T>::RingBuffer(int size)
    {
        assert(size > 0);

        m_buf = new T[size];
        m_buf_size = size;
        m_next_index = 0;
        m_data_count = 0;
    }

    template<class T>
    RingBuffer<T>::~RingBuffer()
    {
        delete[] m_buf;
    }

    template<class T>
    void RingBuffer<T>::push_back(const T& data)
    {
        assert(m_buf);

        m_buf[m_next_index] = data;
        m_next_index = (m_next_index + 1) % m_buf_size;
        if (m_data_count < m_buf_size) m_data_count++;
    }

    template<class T>
    int RingBuffer<T>::data_count() const
    {
        return m_data_count;
    }

    template<class T>
    int RingBuffer<T>::size() const
    {
        return m_buf_size;
    }

    template<class T>
    void RingBuffer<T>::resize(int size)
    {
        assert(size > 0);

        if (m_buf) delete[] m_buf;

        m_buf = new T[size];
        m_buf_size = size;
        m_next_index = 0;
        m_data_count = 0;
    }

    template<class T>
    T& RingBuffer<T>::operator [] (int i)
    {
        assert(i >= 0 && i < m_data_count);

        int index = (m_next_index + i) % m_buf_size;
        if (m_data_count < m_buf_size) index = i;
        return m_buf[index];
    }

    template<class T>
    const T RingBuffer<T>::operator [] (int i) const
    {
        assert(i >= 0 && i < m_data_count);

        int index = (m_next_index + i) % m_buf_size;
        if (m_data_count < m_buf_size) index = i;
        return m_buf[index];
    }

    template<class T>
    RingBufferNumeric<T>::RingBufferNumeric()
        : RingBuffer<T>()
    {
        m_sum = 0;
    }

    template<class T>
    RingBufferNumeric<T>::RingBufferNumeric(int size)
        : RingBuffer<T>(size)
    {
        m_sum = 0;
    }

    template<class T>
    void RingBufferNumeric<T>::push_back(const T& data)
    {
        if (RingBuffer<T>::m_data_count == RingBuffer<T>::m_buf_size) m_sum -= RingBuffer<T>::m_buf[RingBuffer<T>::m_next_index];
        m_sum += data;
        RingBuffer<T>::push_back(data);
    }

    template<class T>
    T RingBufferNumeric<T>::sum()
    {
        return m_sum;
    }

    template<class T>
    double RingBufferNumeric<T>::average()
    {
        if (RingBuffer<T>::m_data_count > 0)
            return m_sum / (double)RingBuffer<T>::m_data_count;
        else
            return 0;
    }

    std::vector<std::string> splitStr(const char* buf, int buf_len, char dlm = ',');

} // End of 'dg'

#endif // End of '__DG_UTILS_UTILITY__'
