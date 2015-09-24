// UGLibrary.h
#ifndef UGLIBRARY_H
#define UGLIBRARY_H

#include <cstdlib>

#include <nanoflann.hpp>

#include "UGQuad.h"
#include "UGVec3.h"
#include "UGUniverse.h"

using namespace nanoflann;


template <class T>
class UGLibrary
{
public:
    UGLibrary(UGUniverse<T> &univ, std::vector<UGDirection <T> > &dirs)
    {
        quadset = new UGQuadSet<T>();

        for(UGDirection<T> dir : dirs) {
            univ.project(dir.theta, dir.phi)
                .calculateQuads(*quadset, dir.theta, dir.phi);
        }

        quadKdtree = new typename UGQuadSet<T>::kdtree(
                7,
                *quadset,
                KDTreeSingleIndexAdaptorParams(1000));
        quadKdtree->buildIndex();

    }

    ~UGLibrary()
    {
        if (quadset) delete quadset;
        if (quadKdtree) delete quadKdtree;
    }

    const UGResult<T> search(UGImage<T>& img, T filterRadius=NULL)
    {
        UGQuadSet<T> qs;
        img.calculateQuads(qs);

        std::cout << "Calculated image quads" << std::endl;

        std::vector<UGResult<T> > results;

        std::cout << "Searching" << std::endl;
#pragma omp parallel for
        for(auto q = qs.quads.begin(); q < qs.quads.end(); q++)
        {
            T error;
            UGQuad<T> *closeq = searchQuad(*q, error);

            UGResult<T> result = UGQuad<T>::quadResult(*q, *closeq);
            result.err = error;
#pragma omp critical(addToResults)
            {
                results.push_back(result);
            }
        }

        std::vector<UGResult<T> > *resultsFiltered;
        if (filterRadius)
        {
            // discard results without close others
            resultsFiltered = new std::vector<UGResult<T> >();
            for(auto r1 = results.begin(); r1 < results.end(); r1++)
            {
                for(auto r2 = r1 + 1; r2 < results.end(); r2++)
                {
                    if(UGDirection<T>::dist(r1->dir,r2->dir)<filterRadius)
                    {
                        resultsFiltered->push_back(*r1);
                        break;
                    }
                }
            }
            std::cout << "Found " << resultsFiltered->size() << " matches" << std::endl;
        }
        else
        {
            resultsFiltered = &results;
        }

        // compute average result
        UGVec3<T> aLocation, aDirVec;
        UGDirection<T> aDirection;
        UGVec2<T> aRollVec;
        T aError, aRoll;
        for(auto res = resultsFiltered->begin(); res < resultsFiltered->end(); res++)
        {
            aLocation += res->loc/res->err;
            aDirVec += res->dir.unit()/res->err;
            aRollVec += UGVec2<T>(res->roll)/res->err;
            aError += 1/res->err;
        }
        aError = 1/aError;
        aLocation = aLocation*aError;
        aDirection = UGDirection<T>(aDirVec);
        aRoll = aRollVec.angle();

        delete resultsFiltered;
        
        return UGResult<T>(
                aLocation,
                aDirection,
                aRoll,
                aError
                );

    }

private:
    UGQuadSet<T>* quadset;
    typename UGQuadSet<T>::kdtree* quadKdtree;

    UGQuad<T>* searchQuad(UGQuad<T> quad, T& error = nullptr)
    {
        size_t ret_index;
        
        nanoflann::KNNResultSet<T> resultSet(1);
        resultSet.init(&ret_index, &error);
        
        T queryPoint[7] = {
            (T)quad.o1,
            (T)quad.o2,
            (T)quad.o3,
            quad.h1,
            quad.h2,
            quad.h3,
            quad.h4
        };

        quadKdtree->findNeighbors(
                    resultSet,
                    &queryPoint[0],
                    nanoflann::SearchParams(1000));

        return &(quadset->quads[ret_index]);
    }
};


#endif
