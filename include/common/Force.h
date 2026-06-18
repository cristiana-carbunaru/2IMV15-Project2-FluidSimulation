#ifndef INC_2IMV15_PROJECT1_FORCE_H
#define INC_2IMV15_PROJECT1_FORCE_H

#endif //INC_2IMV15_PROJECT1_FORCE_H

#pragma once

class Force {
public:
    virtual ~Force() {}
    virtual void addJacobianMultiplication(double* dx, double* dv, double* df, int N) {}
    virtual void apply() = 0;
    virtual void draw() = 0;

    void setEnabled(bool enabled) { m_enabled = enabled; }
    bool isEnabled() const { return m_enabled; }

protected:
    bool m_enabled = true;
};
