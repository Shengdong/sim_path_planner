#include "State.h"

#include <iomanip>
#include <sstream>

const float FLOAT_MAX = std::numeric_limits<float>::max();

State::State()
 : LatticePose()
 , m_g(FLOAT_MAX)
 , m_closed(false)
 , m_status(INIT)
 , m_index(-1)
 , m_isInitialized(false)
{

}

State::State(const State& s)
 : LatticePose(s)
 , m_g(s.m_g)
 , m_closed(false)
 , m_status(INIT)
 , m_index(-1)
 , m_isInitialized(false)
{

}

State::State(const LatticePose& p)
 : LatticePose(p)
 , m_g(FLOAT_MAX)
 , m_closed(false)
 , m_status(INIT)
 , m_index(-1)
 , m_isInitialized(false)
{

}

void
State::clear(void)
{
    m_g = FLOAT_MAX;
    m_closed = false;
    m_index = -1;
    m_isInitialized = false;
    m_status = INIT;
}

float&
State::g(void)
{
    return m_g;
}

float
State::g(void) const
{
    return m_g;
}

float&
State::cost(void)
{
    return m_cost;
}

float
State::cost(void) const
{
    return m_cost;
}

bool&
State::closed(void)
{
    return m_closed;
}

bool
State::closed(void) const
{
    return m_closed;
}

void
State::setIndex(int index)
{
    m_index = index;
}

int
State::getIndex(void) const
{
    return m_index;
}


std::vector<EdgePtr>&
State::incomingEdges(void)
{
    return m_incomingEdges;
}

const std::vector<EdgePtr>&
State::incomingEdges(void) const
{
    return m_incomingEdges;
}

std::vector<EdgePtr>&
State::outgoingEdges(void)
{
    return m_outgoingEdges;
}

const std::vector<EdgePtr>&
State::outgoingEdges(void) const
{
    return m_outgoingEdges;
}

bool&
State::initialized(void)
{
    return m_isInitialized;
}

std::string
State::print(void) const
{
    using namespace std;

    ostringstream oss(ostringstream::out);
    oss << LatticePose::print();

    if (m_g == FLOAT_MAX)
    {
        oss << " g=inf";
    }
    else
    {
        oss << " g=" << m_g;
    }
    return oss.str();
}

