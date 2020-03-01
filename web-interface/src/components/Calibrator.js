import React, {Component} from 'react';
import {Button, Col, Radio, Row, Checkbox, Spin, Icon, Form, Input, Typography} from 'antd';
const { Text } = Typography;

const createTopic = (phone, face, lidar) => `/pixel${phone}/image_${lidar ? 'processed' : ( face ? 'face_detected' : 'raw')}`;

const antIcon = <Icon type="loading" style={{ fontSize: 100 }} spin />;

export default class Calibrator extends Component {
    state = {
        viewValue: 1,
        lidar: false,
        video: false,
        buttonText: 'Launch',
        topic: createTopic(1, true, false),
        loading: false,
        vehicles: true,
        faces: true,
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
        }        
        else {
            this.setState({
                loading: true,
            }, () => {
                const params = [];
                params.push(`camera${this.state.viewValue}`);
                params.push('server');
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
                fetch(`http://localhost:5000/open?launchers=${params.join(',')}`).then(() => {
                    this.setState({
                        video: !this.state.video,
                        topic: createTopic(this.state.viewValue, this.state.faces, this.state.lidar),
                        buttonText: this.state.buttonText === 'Launch' ? 'Back' : 'Launch',
                        loading: false,
                    });
                });
            });
        }
    }

    onFacesChecked = e => {
        this.setState({
            faces: e.target.checked,
        });
    }

    onVehiclesChecked = e => {
        this.setState({
            vehicles: e.target.checked,
        });
    }

    render() {

        let radioStyle = {
            display: 'block',
            height: '60px',
            lineHeight: '60px',
        };

        let smallRadioStyle = {
            display: 'block',
            height: '40px',
            lineHeight: '40px',
        };


        return (
            <div style={{padding: this.props.isLarge ? "50px" : "20px"}}>
                <Form layout={'inline'}>
                    <Row gutter={8}>
                        <Col span={6}>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Text>0</Text>
                            </Form.Item><br/>
                            <Form.Item>
                                <Text>0</Text>
                            </Form.Item>
                        </Col>
                        <Col span={6}>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Text>0</Text>
                            </Form.Item>
                        </Col>
                        <Col span={6}>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Text>1</Text>
                            </Form.Item>
                        </Col>
                        <Col span={6}>
                            <Form.Item>
                                <Text>0</Text>
                            </Form.Item><br/>
                            <Form.Item>
                                <Text>0</Text>
                            </Form.Item><br/>
                            <Form.Item>
                                <Text>0</Text>
                            </Form.Item>
                        </Col>

                    </Row>
                    <Row style={{textAlign: 'right'}}>
                        <Button onClick={this.onLaunch} style={{width: "25%", marginRight: "20px"}} type="primary">Apply</Button>
                        <Button onClick={this.onReset} style={{width: "25%"}} type="primary">Reset</Button>
                    </Row>
                </Form>

                <br/>

                <br/>
                <Form layout={'inline'}>
                    <Row gutter={8}>
                        <Col span={6}>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                        </Col>
                        <Col span={6}>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                        </Col>
                        <Col span={6}>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                        </Col>
                        <Col span={6}>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                            <Form.Item>
                                <Input />
                            </Form.Item>
                        </Col>

                    </Row>
                    <br/>
                    <Row style={{textAlign: 'right'}}>
                        <Button onClick={this.onLaunch} style={{width: "50%"}} type="primary">Calibrate</Button>

                    </Row>
                    <br/>
                    <Row style={{textAlign: 'right'}}>
                        <Button onClick={this.onLaunch} style={{width: "25%", marginRight: "20px"}} type="primary">Apply</Button>
                        <Button onClick={this.onReset} style={{width: "25%"}} type="primary">Reset</Button>
                    </Row>
              </Form>



            </div>
        );
    }
}