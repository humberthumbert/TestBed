import React, {Component} from 'react';
import './App.css';
import {Menu, Layout, Tabs, Icon, Affix, Button, Drawer, Row, Col, Divider, Radio, Input} from 'antd';
import StepComponent from './components/StepComponent';
import LauncherComponent from './components/LauncherComponent';
import Calibrator from "./components/Calibrator";

const {TabPane} = Tabs;

const {Header, Content} = Layout;

function callback(key) {
    console.log(key);
}

class App extends Component {

    state = { visible: false, sourceValue: 1, fileValue: 1, menuKey: '1'};

    showDrawer = () => {
        this.setState({
            visible: true,
        });
    };

    onClose = () => {
        this.setState({
            visible: false,
        });
    };

    onSourceChange = e => {
        this.setState({
            sourceValue: e.target.value,
            menuKey: e.target.value !== 1 ? '1' : this.state.menuKey,
        });
    };
    onFileChange = e => {
        this.setState({
            fileValue: e.target.value,
        });
    };

    onMenuChange = e => {
        this.setState({
            menuKey: e.key,
        });
    }

    render() {
        const radioStyle = {
            display: 'block',
            height: '60px',
            lineHeight: '60px',
        };

        const fileRadioStyle = {
            display: 'block',
            height: '60px',
            lineHeight: '60px',
            marginLeft: '20px'
        };

        return (
            <Layout className="layout">
                <Header style={{position: 'fixed', zIndex: 1, width: '100%'}}>
                    <div className="logo"/>
                    <Menu
                        theme="dark"
                        mode="horizontal"
                        defaultSelectedKeys={['1']}
                        style={{lineHeight: '64px'}}
                        onClick={this.onMenuChange}
                    >
                        <Menu.Item key="1"><Icon type="border"/>Single View</Menu.Item>
                        {this.state.sourceValue === 1 && <Menu.Item key="2"><Icon type="appstore"/>Multiple View</Menu.Item>}
                    </Menu>
                    <Affix style={{position: 'absolute', top: '20vh', right: '0px'}}>
                        <Button type="primary" onClick={this.showDrawer} style={{width: '100px', height: '40px'}}>
                            calibration
                        </Button>
                    </Affix>
                    <Drawer
                        title="Calibration"
                        placement="right"
                        onClose={this.onClose}
                        visible={this.state.visible}
                        width="400px"
                        // drawerStyle={{width: '50vh'}}
                    >
                        <Calibrator/>
                    </Drawer>
                </Header>
                <Content className="content">
                    <Row>
                        <Col span={5}>
                            <div className="content-left-page">
                                <Tabs defaultActiveKey="1">
                                    <TabPane tab="SOURCE" key="1">
                                    </TabPane>
                                </Tabs>
                                <Radio.Group onChange={this.onSourceChange} value={this.state.sourceValue}>
                                    <Radio style={radioStyle} value={1}>
                                        Online
                                    </Radio>
                                    <Radio style={radioStyle} value={2}>
                                        Offline
                                        {/* <br/>
                                        <div style={fileRadioStyle}>
                                            Dataset Path :
                                            <Input
                                                disabled={this.state.sourceValue === 1}
                                                style={{width: 100, marginLeft: 10}}/>
                                        </div> */}
                                    </Radio>
                                </Radio.Group>
                            </div>
                        </Col>
                        <Col span={19}>
                            {this.state.menuKey === '1' ?
                                <div className="content-page">
                                    <Tabs defaultActiveKey="1" onChange={callback}>
                                        <TabPane tab="LAUNCHER" key="1">
                                            <LauncherComponent isLarge={this.state.menuKey === '1'} online={this.state.sourceValue === 1} />
                                        </TabPane>
                                    </Tabs>
                                </div>
                                :
                                <div>
                                    <Row>
                                        <Col span={12}>
                                            <div className="content-small-page">
                                                <Tabs defaultActiveKey="1" onChange={callback}>
                                                    <TabPane tab="LAUNCHER" key="1">
                                                        <LauncherComponent isLarge={this.state.menuKey === '1'} online={this.state.sourceValue === 1} />
                                                    </TabPane>
                                                </Tabs>
                                            </div>
                                        </Col>
                                        <Col span={12}>
                                            <div className="content-small-page">
                                                <Tabs defaultActiveKey="1" onChange={callback}>
                                                    <TabPane tab="LAUNCHER" key="1">
                                                        <LauncherComponent isLarge={this.state.menuKey === '1'} online={this.state.sourceValue === 1} />
                                                    </TabPane>
                                                </Tabs>
                                            </div>
                                        </Col>
                                    </Row>
                                    <Row>
                                        <Col span={12}>
                                            <div className="content-small-page">
                                                <Tabs defaultActiveKey="1" onChange={callback}>
                                                    <TabPane tab="LAUNCHER" key="1">
                                                        <LauncherComponent isLarge={this.state.menuKey === '1'} online={this.state.sourceValue === 1} />
                                                    </TabPane>
                                                </Tabs>
                                            </div>
                                        </Col>
                                        <Col span={12}>
                                            <div className="content-small-page">
                                                <Tabs defaultActiveKey="1" onChange={callback}>
                                                    <TabPane tab="LAUNCHER" key="1">
                                                        <LauncherComponent isLarge={this.state.menuKey === '1'} online={this.state.sourceValue === 1} />
                                                    </TabPane>
                                                </Tabs>
                                            </div>
                                        </Col>
                                    </Row>
                                </div>

                            }
                        </Col>
                    </Row>
                </Content>
            </Layout>
        );
    }
}

export default App;
