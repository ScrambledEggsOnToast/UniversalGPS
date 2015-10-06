// KdTreeBig.h
#ifndef KDTREEBIG_H
#define KDTREEBIG_H
#include "_ug.h"

#include <boost/variant.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/variant.hpp>

#include <eos/portable_iarchive.hpp>
#include <eos/portable_oarchive.hpp>

#include <fstream>
#include <map>
#include <queue>

namespace boost {
namespace serialization {
class access;
}
}

namespace ugps
{
    
    template<
        typename Point, 
        num_ug (*getDim)(const Point&, const size_t&),
        size_t numDims
            >
    class KdTree
    {
    private:
        friend class boost::serialization::access;

        typedef size_t idPart;
        class KdTreeChunkId
        {
        public:
            KdTreeChunkId() : value({}) {}
            KdTreeChunkId(const KdTreeChunkId& parentId, const idPart& childNum) : 
                value(parentId.value) { value.push_back(childNum); }

            string chunkPath() const
            {
                string ret = "ch";
                for(auto i = value.begin(); i < value.end(); i++)
                {
                    ret += "-";
                    ret += to_string(*i);
                }
                return ret;
            }

            friend bool operator==(const KdTreeChunkId& L, const KdTreeChunkId& R)
            {
                return L.value == R.value;
            }

            friend bool operator!=(const KdTreeChunkId& L, const KdTreeChunkId& R)
            {
                return L.value != R.value;
            }

        private:
            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive& ar, const unsigned version)
            {
                ar & value;
            }

            // Root chunk has value = {}. Child chunk x of chunk with value v has value v.push_back(x).
            vector<idPart> value;
        };


        // Nodes
        
        typedef size_t nodeLocation;

        class KdTreeChunkLink;
        class KdTreeLeaf;
        class KdTreeSplit;
        typedef boost::variant<KdTreeLeaf, KdTreeSplit, KdTreeChunkLink> KdTreeNode;

        class KdTreeChunkLink
        {
        public:
            KdTreeChunkLink() : id(KdTreeChunkId()) {}
            KdTreeChunkLink(const KdTreeChunkId& id) : id(id) {}
            KdTreeChunkId id;

            template<typename Archive>
            void serialize(Archive& ar, const unsigned version)
            {
                ar & id;
            }
        };
        class KdTreeLeaf
        {
        public:
            KdTreeLeaf() : points(vector<Point>()) {}
            KdTreeLeaf(const vector<Point>& points) : points(points) 
            {
                if(find(points.begin(),points.end(), Point())!=points.end())
                {
                    LOG("HA");
                }
            }
            vector<Point> points;

            template<typename Archive>
            void serialize(Archive& ar, const unsigned version)
            {
                ar & points;
            }
        };
        class KdTreeSplit
        {
        public:
            KdTreeSplit() {}
            KdTreeSplit(const Point& point, const size_t& dim, const nodeLocation& left, const nodeLocation& right) : point(point), dim(dim), left(left), right(right) {}
            Point point;
            size_t dim;
            nodeLocation left;
            nodeLocation right;

            template<typename Archive>
            void serialize(Archive& ar, const unsigned version)
            {
                ar & point & dim & left & right;
            }
        };

        class KdTreeChunk
        {
            public:
            KdTreeChunk() : id(KdTreeChunkId()), numChildChunks(0), nodes(vector<KdTreeNode>()) {}
            KdTreeChunk(const KdTreeChunkId& id) : id(id), numChildChunks(0), nodes(vector<KdTreeNode>()) {}

            KdTreeNode& root() { return nodes[0]; }

            KdTreeChunkId id;
            size_t numChildChunks;
            vector<KdTreeNode> nodes;

            template<typename Archive>
            void serialize(Archive& ar, const unsigned version)
            {
                ar & id & nodes & numChildChunks;
            }
        };

        void saveChunk()
        {
            boost::filesystem::path savePath = treePath / currentChunk.id.chunkPath();
            boost::filesystem::remove(savePath);
            ofstream ofs(savePath.native());
            eos::portable_oarchive ar(ofs);
            ar & currentChunk;
            LOG("saved chunk " << currentChunk.id.chunkPath());
        }
        void loadChunk(const KdTreeChunkId& id, bool force = false)
        {
            if(force || (id != currentChunk.id))
            {
                boost::filesystem::path loadPath = treePath / id.chunkPath();
                if(boost::filesystem::exists(loadPath))
                {
                    ifstream ifs(loadPath.native());
                    eos::portable_iarchive ar(ifs);
                    currentChunk = KdTreeChunk();
                    ar & currentChunk;
                    LOG("loaded chunk " << currentChunk.id.chunkPath());
                    //logChunk(currentChunk);
                }
            }
        }
        void loadRootChunk(bool force = false) { loadChunk(KdTreeChunkId(), force); }

        template<bool modifying>
        class KdTreeTraverser : public boost::static_visitor<>
        {
        public:
            void go()
            { 
                tree.loadRootChunk();
                traverse(0); 
            }
            KdTreeTraverser(KdTree& tree) : tree(tree) {}

            void traverse(nodeLocation loc)
            {
                apply_visitor(*this, tree.currentChunk.nodes[loc]);
            }

            virtual void handle(const KdTreeSplit&) = 0;
            virtual void handle(const KdTreeLeaf&) = 0;

            KdTree& tree;

            void operator()(const KdTreeChunkLink& link)
            {
                const KdTreeChunkId prevId = tree.currentChunk.id;
                if(modifying) tree.saveChunk();
                tree.loadChunk(link.id);
                traverse(0);
                if(modifying) tree.saveChunk();
                tree.loadChunk(prevId);
            }
            // We must keep a copy of splits and leaves because switching chunks would invalidate references
            // This will only ever take up log(n) space (depth of tree)
            void operator()(const KdTreeSplit node) 
            {
                handle(node);
            }
            void operator()(const KdTreeLeaf node)
            {
                handle(node);
            }
        };

        template<typename Q>
        static num_ug sqrDistance(const Point& p, const Q& q)
        {
            num_ug d = 0.0;
            for(size_t i = 0; i < numDims; i++)
            {
                d += pow(q[i] - getDim(p,i),2);
            }
            return d;
        }

        struct KdResult
        {
            template<typename Q>
            KdResult(const Point& pt, const Q& q) : point(pt) { 
                sqrDist = sqrDistance<Q>(pt,q);
            }
            Point point;
            num_ug sqrDist;
        };

        template<typename Q>
        struct KdResults
        {
        public:

            KdResults(const size_t& maxSize, const Q& q) :
                maxSize(maxSize), vec(vector<KdResult>()), q(q) {}

            void insert(const Point& p)
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
            const Q q;
        };

        class NodesPrinter : boost::static_visitor<>
        {
        public:
            NodesPrinter(const vector<KdTreeNode>& nodes) : indent(0), nodes(nodes) {}
            void print()
            {
                apply_visitor(*this,nodes[0]);
            }
            void operator()(const KdTreeLeaf& leaf)
            {
                LOG(string(indent,'|')<<"leaf("<<leaf.points.size()<<")");
            }
            void operator()(const KdTreeSplit& split)
            {
                LOG(string(indent,'|')<<"split");
                indent++;
                apply_visitor(*this,nodes[split.left]);
                apply_visitor(*this,nodes[split.right]);
                indent--;
            }
            void operator()(const KdTreeChunkLink& chunk)
            {
                LOG(string(indent,'|')<<"link to "<<chunk.id.chunkPath());
            }
            size_t indent;
            const vector<KdTreeNode>& nodes;
        };

        static void logChunk(const KdTreeChunk& chunk)
        {
            LOG(chunk.id.chunkPath() << " =");
            NodesPrinter p(chunk.nodes);
            p.print();
        }

        template<typename Q>
        class KdTreeNN : KdTreeTraverser<false>
        {
            using KdTreeTraverser<false>::traverse;
            void handle(const KdTreeLeaf& leaf)
            {
                for(auto p = leaf.points.begin(); p < leaf.points.end(); p++)
                {
                    results.insert(*p);
                }
            }
            void handle(const KdTreeSplit& split)
            {
                nodeLocation onside, offside;
                num_ug axisDisplacement = q[split.dim] - getDim(split.point, split.dim);
                if(axisDisplacement < 0)
                {
                    onside = split.left;
                    offside = split.right;
                }
                else
                {
                    onside = split.right;
                    offside = split.left;
                }

                traverse(onside);

                results.insert(split.point);

                if((results.size() != nn) || (pow(axisDisplacement, 2) < results.top().sqrDist))
                {
                    traverse(offside);
                }
            }

            const Q q;
            const size_t nn;
        public:
            KdTreeNN(KdTree& tree, const Q& q, const size_t& nn) :
                KdTreeTraverser<false>(tree), q(q), nn(nn), results(KdResults<Q>(nn,q)) { }
            KdResults<Q> results;
            using KdTreeTraverser<false>::go;
        };

        template <typename Q>
        class KdTreeRadius : boost::static_visitor<>
        {
        public:
            KdTreeRadius(KdTree& tree, const num_ug& sqrRadius, const Q& q) :
                tree(tree), sqrRadius(sqrRadius), q(q), results(vector<KdResult>()) {}

            void search()
            {
                chunksToSearch.push(KdTreeChunkId());
                while(!chunksToSearch.empty())
                {
                    tree.loadChunk(chunksToSearch.front());
                    traverse(0);
                    chunksToSearch.pop();
                }
            }

            void traverse(nodeLocation loc)
            {
                apply_visitor(*this, tree.currentChunk.nodes[loc]);
            }

            void operator()(const KdTreeLeaf& leaf)
            {
                for(auto p = leaf.points.begin(); p < leaf.points.end(); p++)
                {
                    KdResult res(*p,q);
                    if(res.sqrDist < sqrRadius) results.push_back(res);
                }
            }
            void operator()(const KdTreeSplit& split)
            {
                nodeLocation onside, offside;
                num_ug axisDisplacement = q[split.dim] - getDim(split.point, split.dim);
                if(axisDisplacement < 0)
                {
                    onside = split.left;
                    offside = split.right;
                }
                else
                {
                    onside = split.right;
                    offside = split.left;
                }

                traverse(onside);
                if(pow(axisDisplacement,2) <= sqrRadius) traverse(offside);
            }
            void operator()(const KdTreeChunkLink& link)
            {
                chunksToSearch.push(link.id);
            }

            KdTree& tree;
            const num_ug sqrRadius;
            const Q q;
            vector<KdResult> results;
            queue<KdTreeChunkId> chunksToSearch;
        };
                    

        template<typename Iterator>
        struct KdTreeBuilder
        {
        private:
            struct ChunkToBuild
            {
                ChunkToBuild(const KdTreeChunkId& id, const Iterator& begin, const Iterator& end) :
                    id(id), begin(begin), end(end) {}
                KdTreeChunkId id;
                Iterator begin;
                Iterator end;
            };
        public:
            KdTreeBuilder(KdTree& tree, const size_t& chunkMaxSize, const size_t& leafMaxSize) : tree(tree), chunkMaxSize(chunkMaxSize), leafMaxSize(leafMaxSize), chunkSize(0), dim(0) 
            {
                random_device rd;
                mt = mt19937(rd());                
                dirDistr = uniform_int_distribution<int>(0,1);
            }

            void build(Iterator begin, Iterator end)
            {
                chunksToBuild.push(ChunkToBuild(KdTreeChunkId(), begin, end));
                while(!chunksToBuild.empty())
                {
                    tree.currentChunk = KdTreeChunk(chunksToBuild.front().id);
                    chunkSize = 0;
                    tree.currentChunk.nodes.resize(1);
                    buildNode(0,chunksToBuild.front().begin,chunksToBuild.front().end);
                    //logChunk(tree.currentChunk);
                    tree.saveChunk();
                    chunksToBuild.pop();
                }
            }
        private:
            void buildNode(const nodeLocation& loc, const Iterator& begin, const Iterator& end)
            {
                if(chunkSize > chunkMaxSize)
                {
                    const KdTreeChunkId newId(tree.currentChunk.id, loc);
                    tree.currentChunk.nodes[loc] = KdTreeChunkLink(newId);
                    chunksToBuild.push(ChunkToBuild(newId,begin,end));
                }
                else if(static_cast<size_t>(distance(begin,end)) <= leafMaxSize)
                {
                    tree.currentChunk.nodes[loc] = KdTreeLeaf(vector<Point>(begin,end));
                    chunkSize += distance(begin, end);
                }
                else
                {
                    dim++; if(dim==numDims) dim = 0;

                    const size_t m = distance(begin,end)/2;
                    nth_element(begin, begin+m, end, 
                            [this](const Point& L, const Point& R) -> bool 
                            {
                                return getDim(L,dim) < getDim(R,dim);
                            });

                    const nodeLocation left = tree.currentChunk.nodes.size();
                    const nodeLocation right = tree.currentChunk.nodes.size()+1;
                    tree.currentChunk.nodes[loc] = KdTreeSplit(*(begin+m), dim, left, right);

                    tree.currentChunk.nodes.resize(tree.currentChunk.nodes.size()+2);
                    
                    chunkSize++;
                    
                    buildNode(left, begin, begin+m);
                    buildNode(right, begin+m+1, end);
                    
                    if(dim==0) dim = numDims; dim--;
                }
            }

            KdTree& tree;

            const size_t chunkMaxSize;
            const size_t leafMaxSize;

            mt19937 mt;
            std::uniform_int_distribution<int> dirDistr;
            size_t chunkSize;
            size_t dim;
            queue<ChunkToBuild> chunksToBuild;
        };

        const boost::filesystem::path treePath;
        KdTreeChunk currentChunk;

    public:

        KdTree(const boost::filesystem::path& treePath) : treePath(treePath), currentChunk(KdTreeChunk()) { loadRootChunk(true); }

        KdTree(const boost::filesystem::path& treePath, vector<Point> points, const size_t& chunkMaxSize, const size_t& leafMaxSize) : treePath(treePath), currentChunk(KdTreeChunk())
        {
            KdTreeBuilder<typename vector<Point>::iterator> builder(*this, chunkMaxSize, leafMaxSize);
            builder.build(points.begin(), points.end());
        }
        
        template<typename Q>
        vector<KdResult> knn(const Q& q, const size_t& nn)
        {
            KdTreeNN<Q> searcher(*this, q, nn);
            searcher.go();
            return searcher.results.get();
        }

        template<typename Q>
        vector<KdResult> rnn(const Q& q, const num_ug& sqrRadius)
        {
            KdTreeRadius<Q> searcher(*this, sqrRadius, q);
            searcher.search();
            return searcher.results;
        }

    };

}

#endif
