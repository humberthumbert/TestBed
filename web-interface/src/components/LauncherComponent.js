import React, {Component} from 'react';
import {Button, Col, Radio, Row, Checkbox, Spin, Icon, Input } from 'antd';

const createTopic = (phone, face, lidar, online = true) => {
    if (online) {
        let part2 = 'raw';
        if (lidar && face) {
            part2 = 'face_processed';
        } else if (lidar && !face) {
            part2 = 'processed';
        } else if (face && !lidar) {
            part2 = 'face_detected';
        }
        return `/pixel${phone}/image_${part2}`;
    } else {
        switch (phone) {
            case 0:
                return `pc_detection0`;
            case 1:
                return `image_detection2`;
            default:
                return '';
        }
    }
}

const antIcon = <Icon type="loading" style={{ fontSize: 100 }} spin />;

export default class LauncherComponent extends Component {
    state = {
        viewValue: 1,
        lidar: false,
        video: false,
        buttonText: 'Launch',
        topic: createTopic(1, true, false),
        loading: false,
        vehicles: true,
        faces: true,
        offlineCamera: 0,
        datasetPath: '',
    };

    onViewChange = e => {
        this.setState({
            viewValue: e.target.value,
        });
    };

    onLidarChecked = e => {
        this.setState({
            lidar: e.target.checked,
        });
    };

    onReset = () => {
        fetch('http://localhost:5000/stop')
        .then(() => {
            if (this.state.video) {
                this.onStop();
            }
        });
    }

    onStop = () => {
        this.setState({
            video: !this.state.video,
            buttonText: this.state.buttonText === 'Launch' ? 'Back' : 'Launch',
            loading: false,
        });
    }

    onLaunch = e => {
        if (this.state.buttonText === 'Back') {
            this.onStop();
        } else {
            this.setState({
                loading: true,
            }, () => {
                const params = [];
                params.push('server');
                if (this.props.online) {
                    params.push(`camera${this.state.viewValue}`);
                    if (this.state.lidar) {
                        params.push(`velodyne`);
                    }
                    if (this.state.faces) {
                        params.push(`face${this.state.viewValue}`);
                    }

                    if (this.state.lidar && this.state.faces) {
                        params.push(`chained_projection${this.state.viewValue}`);
                    } else if (this.state.lidar){
                        params.push(`projection${this.state.viewValue}`);
                    }
                } else {
                    params.push('offline');
                }
                
                fetch(`http://localhost:5000/open?launchers=${params.join(',')}&offlinepath=${this.state.datasetPath}`).then(() => {
                    this.setState({
                        video: !this.state.video,
                        topic: createTopic(this.props.online ? this.state.viewValue : this.state.offlineCamera, this.state.faces, this.state.lidar, this.props.online),
                        buttonText: this.state.buttonText === 'Launch' ? 'Back' : 'Launch',
                        loading: false,
                    });
                });
            });
        }
    }

    onDatasetPathChanged = e => {
        this.setState({
            datasetPath: e.target.value,
        })
    }

    onFacesChecked = e => {
        this.setState({
            faces: e.target.checked,
        });
    }

    onOfflineCameraChanged = e => {
        this.setState({
            offlineCamera: e.target.value,
        });
    }

    render() {

        let radioStyle = {
            display: 'block'
        };

        return (
            <div style={{padding: this.props.isLarge ? "5%" : "1%"}}>
                { this.state.video &&
                    (<center>
                        <img
                            src={`http://localhost:8080/stream?topic=${this.state.topic}`}
                            width={this.props.online ? (this.props.isLarge ? "40%" : "88%") : "100%"}
                            height={this.props.isLarge ? "30%" : "66%"}
                        />
                    </center>)
                }
                {!this.state.video && <Row>
                    {this.state.loading && (
                        <center><Spin indicator={antIcon} /></center>
                    )}
                     
                    {!(this.state.loading || this.state.video) &&
                    ( <React.Fragment>
                        {this.props.online && (<Col span={8}>
                            View
                            <br/>
                            <br/>
                            <Radio.Group onChange={this.onViewChange} value={this.state.viewValue}>
                                <Radio style= {radioStyle} value={1}>Front</Radio>
                                <Radio style= {radioStyle} value={2}>Right</Radio>
                                <Radio style= {radioStyle} value={3}>Back</Radio>
                                <Radio style= {radioStyle} value={4}>Left</Radio>
                            </Radio.Group>
                        </Col>)}
                        {!this.props.online && <Col span={8}>
                            Offline View
                            <br/>
                            <br/>
                            <Radio.Group onChange={this.onOfflineCameraChanged} value={this.state.offlineCamera}>
                                <Radio style= {radioStyle} value={0}>Lidar</Radio>
                                <Radio style= {radioStyle} value={1}>Camera</Radio>
                            </Radio.Group>
                        </Col>}
                        {this.props.online && (<Col span={8}>
                            Mode
                            <br/>
                            <br/>
                            <Checkbox onChange={this.onLidarChecked} checked={this.state.lidar}>Lidar Enabled</Checkbox>
                            <br/>
                            <Checkbox onChange={this.onFacesChecked} checked={this.state.faces}>Face Detection</Checkbox>
                        </Col>)}
                        {!this.props.online && (<Col span={10}>
                            Parameters
                            <br/>
                            <br/>
                            Dataset Path: <Input size="large" style={{marginLeft: 10}} onChange={this.onDatasetPathChanged}/>
                        </Col>)}
                    </React.Fragment>
                    )}
                </Row>}
                {this.props.isLarge ?
                    <Row style={{marginTop: '5%', textAlign: 'center'}}>
                        <Button onClick={this.onLaunch} style={{width: "25%", marginRight: "5%"}} type="primary">{this.state.buttonText}</Button>
                        <Button onClick={this.onReset} style={{width: "25%"}} type="primary">Reset</Button>
                    </Row> :
                    <Row style={{marginTop: '5%', textAlign: 'right'}}>
                        <Button onClick={this.onLaunch} style={{width: "25%", marginRight: "5%"}} type="primary">{this.state.buttonText}</Button>
                        <Button onClick={this.onReset} style={{width: "25%"}} type="primary">Reset</Button>
                    </Row>
                }
            </div>
        );
    }
}