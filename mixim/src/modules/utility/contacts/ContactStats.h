//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef CONTACTSTATS_H_
#define CONTACTSTATS_H_

class ContactStats {
protected:
	int L3Sent;
	int L3Received;
	int ackSent;
	int ackReceived;
	int bundleSent;
	int bundleReceived;
	int predictionsSent;
	int predictionsReceived;
public:
	ContactStats();
	virtual ~ContactStats();

	int getAckReceived() const
    {
        return ackReceived;
    }

    int getAckSent() const
    {
        return ackSent;
    }

    int getBundleReceived() const
    {
        return bundleReceived;
    }

    int getBundleSent() const
    {
        return bundleSent;
    }

    int getL3Received() const
    {
        return L3Received;
    }

    int getL3Sent() const
    {
        return L3Sent;
    }

    int getPredictionsReceived() const
    {
        return predictionsReceived;
    }

    int getPredictionsSent() const
    {
        return predictionsSent;
    }

    void setAckReceived(int ackReceived)
    {
        this->ackReceived = ackReceived;
    }

    void setAckSent(int ackSent)
    {
        this->ackSent = ackSent;
    }

    void setAckReceived()
	{
		this->ackReceived++;
	}

	void setAckSent()
	{
		this->ackSent++;
	}

    void setBundleReceived(int bundleReceived)
    {
        this->bundleReceived = bundleReceived;
    }

    void setBundleReceived()
	{
		this->bundleReceived++;
	}

    void setBundleSent(int bundleSent)
    {
        this->bundleSent = bundleSent;
    }

    void setBundleSent()
	{
		this->bundleSent++;
	}

    void setL3Received(int l3Received)
    {
        L3Received += l3Received;
    }

    void setL3Received()
	{
		L3Received++;
	}

    void setL3Sent(int l3Sent)
    {
        L3Sent += l3Sent;
    }

    void setL3Sent()
	{
		L3Sent++;
	}

    void setPredictionsReceived(int predictionsReceived)
    {
        this->predictionsReceived = predictionsReceived;
    }

    void setPredictionsSent(int predictionsSent)
    {
        this->predictionsSent = predictionsSent;
    }

};

#endif /* CONTACTSTATS_H_ */
