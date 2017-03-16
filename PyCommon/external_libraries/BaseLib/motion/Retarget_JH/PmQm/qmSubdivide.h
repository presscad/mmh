
#ifndef __QM_SUBDIVIDE_H
#define __QM_SUBDIVIDE_H



m_real QmCardinalFunction( int, int, m_real*, m_real );

void QmSubdivideWeights( int, m_real* );

void QmInterpolatorySubdivide( int, jhm::vector*, jhm::vector*, char* );
void QmInterpolatorySubdivide( int, jhm::quater*, jhm::quater*, char* );

void QmInterpolatorySubdivide( int, jhm::vector*, jhm::vector*, char*, m_real* );
void QmInterpolatorySubdivide( int, jhm::quater*, jhm::quater*, char*, m_real* );

void QmBsplineSubdivide( int, jhm::vector*, jhm::vector* );
void QmBsplineSubdivide( int, jhm::quater*, jhm::quater* );

#endif

