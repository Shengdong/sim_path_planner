#include "LatticePose.h"
#include "PrimitivePath.h"

class State;

typedef boost::shared_ptr<State> StatePtr;
typedef boost::shared_ptr<const State> StateConstPtr;

class Edge
{
public:
   Edge()
    : m_traversalCost(0.0f)
   {

   }

   float& traversalCost(void)
   {
       return m_traversalCost;
   }
   float traversalCost(void) const
   {
       return m_traversalCost;
   }
   
   StatePtr& stateIn(void)
   {
       return m_stateIn;
   }
   StatePtr stateIn(void) const
   {
       return m_stateIn;
   }
   StatePtr& stateOut(void)
   {
       return m_stateOut;
   }
   StatePtr stateOut(void) const
   {
       return m_stateOut;
   }
 
   PrimitivePathPtr& path(void)
   {
       return m_path;
   }

   PrimitivePathPtr path(void) const
   {
       return m_path;
   }

private:
   float m_traversalCost;
   StatePtr m_stateIn;
   StatePtr m_stateOut;
   PrimitivePathPtr m_path;

};

typedef boost::shared_ptr<Edge> EdgePtr;
typedef boost::shared_ptr<const Edge> EdgeConstPtr;

class State: public LatticePose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    State();
    State(const State& s);
    State(const LatticePose& p);

    void clear(void);

    float& g(void);
    float g(void) const;

    float& cost(void);
    float cost(void) const;

    bool& closed(void);
    bool closed(void) const;

    void setIndex(int index);
    int getIndex(void) const;

    std::vector<EdgePtr>& incomingEdges(void);
    const std::vector<EdgePtr>& incomingEdges(void) const;

    std::vector<EdgePtr>& outgoingEdges(void);
    const std::vector<EdgePtr>& outgoingEdges(void) const;

    bool& initialized(void);

    std::string print(void) const;
    bool isclosed(void)
    {
       return m_status == CLOSE;
    }
    void setclosed(void)
    {
       m_status = CLOSE;
    }

    bool isopen(void)
    {
       return m_status == OPEN;
    }
    void setopen(void)
    {
       m_status = OPEN;
    }


    bool isunexplored(void)
    {
       return m_status == INIT;
    } 
    void setunexplored(void)
    {
       m_status = INIT;
    }

    StatePtr& previous(void)
    {
       return m_previous;
    }  

    StatePtr previous(void) const
    {
       return m_previous;
    }

private:

    enum Status
    {
        INIT = 0,
        OPEN = 1,
        CLOSE = 2,
    };
    Status m_status;
    float m_g; // cost from s to the start
    float m_cost;

    bool m_closed;

    int m_index;

    std::vector<EdgePtr> m_incomingEdges;
    std::vector<EdgePtr> m_outgoingEdges;
 
    StatePtr m_previous;
    bool m_isInitialized;
};

class CompareState
{
public:
    bool operator()(StatePtr x, StatePtr y) const
    {
        return x->cost() > y->cost();
    }
};

