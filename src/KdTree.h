// KdTree.h
#ifndef KDTREE_H
#define KDTREE_H
#include "_ug.h"

#include <boost/variant.hpp>
#include <queue>
#include <algorithm>

namespace ugps
{

    template<typename P, num_ug (*getValue)(const P&,const size_t&), size_t dims, size_t chunkDepth, size_t listSize>
    class KdTree
    {
    private:
        typedef typename vector<P>::iterator kditerator;

        struct Deleter : public boost::static_visitor<void>
        {
            Deleter() {}
            template<class T>
            inline void operator()(T* t) const { delete t; }
        };

        template<typename Q>
        static num_ug sqrDistance(const P& p, const Q& q)
        {
            num_ug d;
            for(size_t i = 0; i < dims; i++)
            {
                d += pow(q[i] - getValue(p,i),2);
            }
            return d;
        }

        template<typename Q>
        struct KdDistanceCompare
        {
            KdDistanceCompare(const Q& q): q(q) {}

            bool operator()(const P& a, const P& b) const
            {
                return sqrDistance(a, q) < sqrDistance(b, q);
            }

            const Q& q;
        };
        
        struct KdTreeNodeList
        {
            KdTreeNodeList(const kditerator& begin, const kditerator& end) : 
                points(vector<P>(begin, end)) {};

            vector<P> points;
            
            size_t size() const
            {
                return points.size();
            }
        };

        struct KdTreeNodeSplit
        {
            typedef boost::variant<KdTreeNodeSplit*, KdTreeNodeList*> KdTreeNode;

            KdTreeNodeSplit(const P& point,
                    const KdTreeNode& left,
                    const KdTreeNode& right) :
                point(point), left(left), right(right) {}

            P point;
            KdTreeNode left;
            KdTreeNode right;

            ~KdTreeNodeSplit()
            {
                Deleter d;
                apply_visitor(d, left);
                apply_visitor(d, right);
            }
        };

        struct KdDimensionCompare
        {
            KdDimensionCompare(int dim) : dim(dim) {}

            bool operator()(const P& a, const P& b) const
            {
                return getValue(a,dim) < getValue(b,dim);
            }

            int dim;
        };

        typedef typename KdTreeNodeSplit::KdTreeNode KdTreeNode;

        struct KdResult
        {
            template<typename Q>
            KdResult(const P& point, const Q& q) : point(point), sqrDist(sqrDistance<Q>(point,q)) {}
            P point;
            num_ug sqrDist;
        };

        template<typename Q>
        struct KdResults
        {
        public:

            KdResults(const size_t& maxSize, const Q& q) :
                maxSize(maxSize), vec(vector<KdResult>()), q(q) {}

            void insert(P p)
            {
                KdResult res(p,q); 
                if(vec.size() < maxSize)
                {
                    vec.push_back(res);
                    upHeap(vec.size()-1);
                }
                else if(res.sqrDist < vec[0].sqrDist)
                {
                    vec[0] = res;
                    maxHeapify(0);
                }
            }

            vector<KdResult> const& get()
            {
                return vec;
            }

            size_t size() const
            {
                return vec.size();
            }

            KdResult const& top()
            {
                return vec[0];
            }

        private:
            void upHeap(const size_t& i)
            {
                if(i > 0)
                {
                    const size_t parent = (i-1)/2;
                    if(vec[parent].sqrDist < vec[i].sqrDist)
                    {
                        swap(vec[parent], vec[i]);
                        upHeap(parent);
                    }
                }
            }

            void maxHeapify(const size_t& i)
            {
                if(i < vec.size())
                {
                    const size_t left = 2*i+1;
                    const size_t right = 2*i+2;
                    size_t largest = i;
                    if(left < vec.size())
                    {
                        if (vec[left].sqrDist > vec[largest].sqrDist)
                            largest = left;

                        if(right < vec.size())
                        {
                            if (vec[right].sqrDist > vec[largest].sqrDist)
                                largest = right;
                        }
                    }

                    if(largest != i)
                    {
                        swap(vec[i],vec[largest]);
                        maxHeapify(largest);
                    }

                }
            }

            const size_t maxSize;
            vector<KdResult> vec;
            const Q& q;
        };

        template<typename Q>
        struct KdNearestNeighbour : public boost::static_visitor<>
        {
            KdNearestNeighbour(const Q& q, const size_t& nn, KdTree& tree) : 
                q(q), nn(nn), distComp(KdDistanceCompare<Q>(q)), dim(0),
                results(KdResults<Q>(nn, q)), tree(tree) {}

            void operator()(KdTreeNodeList* const& list)
            {
                kditerator end;
                if(nn < list->size())
                {
                    end = list->points.begin() + nn;
                    nth_element(list->points.begin(), end, list->points.end(), distComp);
                }
                else end = list->points.end();

                for(auto p = list->points.begin(); p < end; p++)
                {
                    results.insert(*p);
                }
            }

            void operator()(KdTreeNodeSplit* const& split)
            {
                KdTreeNode nextNode, maybeNode;
                num_ug axisDisplacement = q[dim] - getValue(split->point, dim);

                if(axisDisplacement < 0)
                {
                    nextNode = split->left;
                    maybeNode = split->right;
                }
                else
                {
                    nextNode = split->right;
                    maybeNode = split->left;
                }

                continueSearch(nextNode);

                results.insert(split->point);

                if((results.size() != nn) || (pow(axisDisplacement,2) < results.top().sqrDist))
                {
                    continueSearch(maybeNode);
                }
            }

            void continueSearch(const KdTreeNode &node)
            {
                dim = (dim + 1) % dims;
                boost::apply_visitor(*this,node);
                if(dim == 0) dim = dims - 1;
                else dim = (dim - 1) % dims;
            }

            const Q& q;
            const size_t nn;
            const KdDistanceCompare<Q> distComp;
            
            size_t dim;

            KdResults<Q> results;

            KdTree& tree;

        };

        KdTreeNode buildNode(const kditerator& begin, const kditerator& end, const size_t& n)
        {
            size_t size = distance(begin,end);
            if(size <= listSize)
            {
                return new KdTreeNodeList(begin, end);
            }
            else
            {
                const size_t m = size / 2;
                nth_element(begin, begin + m, end, KdDimensionCompare(n));

                const size_t newN = (n + 1) % dims;
                return new KdTreeNodeSplit(*(begin+m),
                        buildNode(begin, begin + m, newN),
                        buildNode(begin + (m + 1), end, newN));
            }
        }

        KdTreeNode root;
    public:
        KdTree(vector<P> pts) : root (buildNode(pts.begin(),pts.end(),0)) {}
        ~KdTree() { apply_visitor(Deleter(), root); }

        template<typename Q>
        vector<KdResult> nearestNeighbours(const Q& q, const size_t& nn)
        {
            KdNearestNeighbour<Q> searcher(q, nn, *this);
            boost::apply_visitor(searcher, root);
            
            vector<P> ret;
            P pt;

            return searcher.results.get();
        }

    };

}

#endif
