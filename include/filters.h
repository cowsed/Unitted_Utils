#pragma once
#include <stdint.h>
#include <array>

/**
 * Circular Buffer
 *
 */
template <int M, int L, int T, int A, int I, std::size_t N>
class CircularBuffer
{
public:
    typedef unit::Quantity<M, L, T, A, I> value_T;

    /**
     * Construct a circular buffer with an initial value of 0
     */
    CircularBuffer() : index(0)
    {
        for (int i = 0; i < N; i++)
        {
            buffer[i] = 0;
        }
    }
    /**
     * Construct a circular buffer with a specified initial value
     */
    CircularBuffer(value_T defualt) : index(0)
    {
        for (int i = 0; i < N; i++)
        {
            buffer[i] = defualt;
        }
    }

    /**
     * Add a new value to the buffer
     * this will overwrite the oldest value
    */
    void add(value_T val)
    {
        buffer[index] = val;
        index++;
        index %= N;
    }

    /**
     * get a value from the buffer
     * for a buffer size of N, index 0 is the oldest and index N-1 is the newest value
    */
    value_T const get(const int i)  { return buffer[(index + i) % N]; }

    int const size() { return N; }

private:
    std::array<value_T, N> buffer;
    int index = 0;
};

// << operator for circular buffer to enable printing with std::cout
template <int M, int L, int T, int A, int I, std::size_t N>
std::ostream &operator<<(std::ostream &os, const CircularBuffer<M, L, T, A, I, N> &circ)
{
    os << "[ ";
    for (int i = 0; i < N; i++)
    {
        os << circ.get(i).getValue() << " ";
    }
    os << " ]";
    return os;
}

/**
 * Simple Moving Average
 */
template <int M, int L, int T, int A, int I, std::size_t N>
class MovingAverage
{
    typedef unit::Quantity<M, L, T, A, I> sample_type;

public:
    /**
     * Construct a moving average with the templated size and a default value of 0
     */
    MovingAverage() : buf(), current_sum(0.0) {}
    /**
     * Construct a moving average with the templated size and a specified default value
     * @param default_val the default value of the moving average
     */
    MovingAverage(sample_type default_val) : buf(default_val), current_sum(((double)N) * default_val) {}

    /**
     * Adds a sample to the moving average
     * @param new_sample the sample to add to the moving average
     */
    void add_sample(sample_type new_sample)
    {
        sample_type oldest_sample = buf.get(0);
        current_sum -= oldest_sample;
        current_sum += new_sample;
        buf.add(new_sample);
    }
    /**
     * Gets the value of the moving average at this point
     * @return the current calculated average
     */
    sample_type get_sample() const
    {
        return current_sum / ((double)N);
    }

private:
    CircularBuffer<M, L, T, A, I, N> buf;
    sample_type current_sum;
};
