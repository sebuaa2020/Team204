//
// Created by yspjack on 2020/5/20.
//

#ifndef SRC_TTS_ENGINE_H
#define SRC_TTS_ENGINE_H

#include <string>

/** \brief TTSEngine interface
 */
class TTSEngine {
public:
    /** \brief
     * \param str String
     */
    virtual void speak(const std::string &str) = 0;
};

#endif //SRC_TTS_ENGINE_H
